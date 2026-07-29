# Technical Specifications: 3D LiDAR Data Acquisition System (Unitree L2)

This document outlines the system developed during the work placement: its objective, the hardware, how the software works module by module, and the operating procedure. 

## Glossary

| Term | Definition |
|---|---|
| **LiDAR** | A sensor that measures distances using the time-of-flight of laser pulses; when rotating, it produces a 3D point cloud. |
| **IMU** | Inertial measurement unit: accelerometer + gyroscope, provides the sensor’s orientation. |
| **UDP** | A lightweight, connectionless network protocol, suitable for high-frequency sensor streams. |
| **CRC-32** | A checksum that detects message corruption. |
| **Payload** | The ‘payload’ portion of a message, between the header and the trailer. |
| **Quaternion** | A mathematical representation of a 3D orientation with four components, free from singularities. |
| **Roll / Pitch / Yaw** | Roll, pitch, yaw: the three standard orientation angles. |
| **Hough transform** | An algorithm for detecting geometric shapes (in this case, straight lines) in an image. |
| **Deskew** | Correction of distortion in a scan caused by sensor movement during acquisition. |
| **Vectorisation (NumPy)** | Calculation applied to entire arrays in one go, orders of magnitude faster than a Python loop. |
| **systemd / service** | Linux mechanism for automatically launching a programme at boot time. |

## 1. Purpose of the system

The system captures a **3D point cloud** of the environment using a rotating LiDAR, and then:

1. Reconstruct the 3D coordinates of each measured point
2. Automatically detect **the ceiling**, **the walls** and **the corners** of the room
3. Calculate the height of the sensor relative to the ceiling
4. Display all this information in an **interactive web interface** accessible from any browser on the local network

The whole system operates **completely autonomously** on an embedded Jetson board: it starts up automatically when powered on, and data acquisition and visualisation can be stopped using two physical buttons, without the need for a screen or keyboard.

## 2. Hardware

| Component | Role |
|---|---|
| **Unitree 4D L2 LiDAR** | Rotating laser sensor. Provides scan lines of 300 distance and intensity measurements, as well as an integrated **IMU** (accelerometer, gyroscope, orientation). 12 V power supply via the mains adapter |
| **Jetson Orin Nano Super** (Yahboom board) | On-board computer running all the software |
| **Ethernet connection (UDP)** | Sole communication channel used with the LiDAR. The sensor’s TTL/UART serial connection is **not** used |
| **Two buttons on GPIO** | Physical pin 11: stop data acquisition. Physical pin 13: stop visualisation. Wiring diagram: `img/electrical_circuit_diagram.png` |

**Network addressing** (fixed, hard-coded in the software):

| Device | IP | UDP port |
|---|---|---|
| LiDAR | `192.168.1.62` | 6101 (command reception) |
| Jetson | `192.168.1.100` | 6201 (data reception) |

The full procedure for configuring the network and GPIO pins is detailed in the `README.md` file in the repository.

## 3. Software overview

The software is written in **Python** (in the `src/` folder). It is a complete reimplementation of the official Unitree SDK (provided in C++ in `original_sdk/`, which has been retained for reference): the sensor’s network protocol has been decoded and rewritten in Python to give full control over the entire chain.

```
 GPIO buttons ──────────────┐ (parallel monitoring)
                            ▼
 LiDAR ══ UDP ══▶ [1] Acquisition ──▶ [2] Processing ──▶ [3] Web visualisation
                   (lidar/…)            (process.py)        (visualisation.py)
                       │
                       └──▶ .npy save (can be replayed without the sensor)
```

| File | Role |
|---|---|
| `src/main.py` | Orchestrator: arguments, launching GPIO monitoring, acquisition → processing → visualisation workflow |
| `src/lidar/lidar.py` | Network protocol: constructing commands sent to the LiDAR, decoding received packets |
| `src/lidar/lidar_manager.py` | Acquisition loop: starts the sensor, accumulates data, saves it |
| `src/gpio.py` | Monitoring of the two physical buttons (separate process) |
| `src/load.py` | Reloading of saved data (replay mode, without sensor) |
| `src/process.py` | All calculations: 3D reconstruction, ceiling, walls, corners |
| `src/visualisation.py` | Web interface (Dash/Plotly) |
| `src_alexander/` | Separate pipeline for replaying CSV files from a **different** LiDAR |
| `service/` | systemd service file for automatic start-up at boot |

## 4. Acquisition: the network protocol (`src/lidar/`)

### 4.1 Message format

The LiDAR communicates via **UDP datagrams**. Each message (in both directions) follows the same format:

```
┌─────────── Header (12 bytes) ──────────┬── Data ─┬─── Trailer (12 bytes) ─────┐
│ 55 AA 05 0A │ packet type │ total size │ payload │ CRC-32 │ reserved │ 00 FF  │
└────────────────────────────────────────┴─────────┴────────────────────────────┘
```

- The first 4 bytes (`55 AA 05 0A`) are a fixed signature that identifies a Unitree frame.
- The **CRC-32** is a checksum: the recipient recalculates this value and rejects the message if it does not match (protection against corruption).

**Types of packets used:**

| Type | Direction | Content | Size |
|---|---|---|---|
| 100 | Jetson → LiDAR | Command (start / standby / restart) | — |
| 101 | LiDAR → Jetson | Command acknowledgement (success or error code) | 40 o |
| 102 | LiDAR → Jetson | One scan line: 300 distances + 300 intensities + calibration parameters | 1044 o |
| 104 | LiDAR → Jetson | IMU measurements: orientation (quaternion), angular velocities, accelerations | 80 o |
| 2002 | Jetson → LiDAR | Configuration of operating mode | — |

### 4.2 Key findings: the official documentation is incorrect

The official Unitree SDK is supplied as a compiled library (closed-source code) accompanied by commented header files. To rewrite the protocol in Python, we based our work on the structure provided in the official SDK and carried out extensive testing. We discovered that one comment in the official header files turned out to be **incorrect** (this may need to be verified by capturing real frames, by pointing to a monitored local port):

**The CRC-32 covers only the data (payload), not the header.** The official comment states ‘crc check of header and data’ — this is incorrect. Calculating the CRC on the header plus data causes the sensor to systematically reject commands (`ACK_CRC_ERROR`). This was the project’s long-standing blocking bug; it has been fixed and **validated on actual hardware on 18 June 2026** (commands accepted, no further CRC errors).

### 4.3 Acquisition procedure

1. Open the UDP socket on `192.168.1.100:6201` (20-second timeout).
2. Send the operating mode configuration (default mask: all zeros = standard 3D mode, IMU active).
3. Send the **start** command (the sensor begins to rotate), then wait for a fixed period of 20 seconds for stabilisation (to allow the motor to come up to full speed).
4. Reception loop: each datagram is identified by its type and decoded; scan lines (102) and IMU measurements (104) are accumulated in memory.
5. Pressing the **pin 11 button** (monitored by a parallel process) stops the loop.
6. The **standby** command is sent, the socket is closed, and the raw data is **saved** to two timestamped files: `data/points_YYYYMMDD_HHMMSS.npy` and `data/imu_….npy`. These files allow a session to be replayed in full without the sensor (`--file-name`).

## 5. Data processing (`src/process.py`)

### 5.1 3D Reconstruction

The sensor does not transmit X/Y/Z coordinates but rather **raw distances** along known directions (line angle + angular step), accompanied in each packet by its **factory calibration parameters** (angle offsets, distance scale, distances between mechanical axes, etc.). The reconstruction applies the geometric formulas from the official C++ SDK, transposed into **NumPy vectorised** calculations (the entire data set is processed in a single block, without Python loops — necessary to maintain the processing speed on Jetson).

Filters applied during processing: zero measurements removed, distances outside the sensor’s valid range removed, and a safety cut-off at ±30 m (to eliminate outliers).

### 5.2 Orientation data (IMU)

The quaternions provided by the sensor (a mathematical representation of orientation, insensitive to singularities) are converted into readable **roll / pitch / yaw** angles (in degrees).

Special consideration: the sensor transmits the quaternion in the order `(w, x, y, z)`, whereas the SciPy library expects `(x, y, z, w)`.

### 5.3 Ceiling Detection

Assumption: the ceiling is the dominant horizontal surface above the sensor. Points are grouped by altitude Z (rounded to the nearest centimetre), with each point **weighted by its vertical proximity to the sensor** (a point directly above the sensor is more reliable than a point skimming the sensor at 15 m). The altitude at which the cumulative weight is at its maximum is taken as the ceiling, and all points within 40 cm below this level (and above it) are removed from the point cloud.

### 5.4 Detection of walls and corners

1. **Walls**: a horizontal ‘slice’ is isolated at mid-height between the floor (estimated at the 5th percentile of altitudes) and the ceiling, ±20%. Viewed from above, this slice outlines the room.
2. **Lines**: the slice is projected onto a 2D grid (5 cm per square), on which a **Hough transform** (a standard algorithm for detecting straight lines in an image) extracts up to 10 dominant lines: these are the walls.
3. **Corners**: each pair of non-parallel lines is intersected; the intersections located within the cloud’s footprint are candidate corners, deduplicated to within 50 cm.

### 5.5 Motion correction (deskew) — implemented but disabled

A full scan takes some time: if the subject moves during the rotation, the point cloud becomes distorted. A correction using the IMU (interpolation of angular velocities and accelerations at the time of each point) is implemented in `process.py`, but is **currently disabled** in the visualisation (call commented out). Without it, the results are only reliable when the sensor is **stationary**.

## 6. Visualisation (`src/visualisation.py`)

Web interface built using **Dash/Plotly**, served on port 8050 (URL displayed on launch, accessible from anywhere on the local network):

- Interactive **3D scatter plot** (rotate/zoom with the mouse), coloured by laser return intensity; sampled at a maximum of 200,000 points on screen to ensure smooth performance
- **Time slider** to browse the session in segments (fixed step size, `--delta`), or a ‘Show all data points’ toggle to display everything
- **Markers**: LiDAR position, nearest and furthest points (with distances), **detected corners** (diamonds)
- **IMU panel**: ceiling height (reference 3.00 m), average accelerations and angles for the displayed segment; full acceleration + roll/pitch/yaw plots
- Dark or light theme (`--dark-mode`).

The web server runs in a separate thread: pressing the **pin 13 button** shuts it down properly, which terminates the programme. 

## 7. Analysis

### 7.1 Automatic start-up (on-board nominal mode)

A systemd service `pin.service` (see `service/boot_service.txt` and README) launches `src/main.py` when the Jetson boots. Operator scenario, without a display:

1. Power on the Jetson and the LiDAR (12 V) → data acquisition starts automatically.
2. Press the **pin 11** button → end of data acquisition, save, calculations, start the web server.
3. View the results in a browser on the local network (port 8050).
4. Press the button on **pin 13** → the server and programme shut down properly.

### 7.2 Manual launch and replay

```
cd src/
python main.py                                # live data acquisition
python main.py --file-name _20260618_143000   # replay a saved session
```

Arguments: 
- `--file-name` (replay .npy)
- `--delta` (time cursor step)
- `--save`
- `--port`
- `--max-pts`
- `--dark-mode`

The `src_alexander/` folder provides an equivalent pipeline for CSV files from the other LiDAR (using `split_csv.py` to split large files). Details in the README. 

## 8. Known limitations and open issues

### 8.1 Probable regression: ‘work mode’ packet type

The mode-change command may be using packet type **2002** on the network, rather than **107** as specified by the official constant `LIDAR_WORK_MODE_CONFIG_PACKET_TYPE`. It is possible that, as things stand, the configuration command sent at start-up is likely to be **ignored by the sensor**.

There are no visible symptoms at present as the mask sent is the default mask (standard 3D mode) — but any future attempt to change modes (2D, wide FOV, IMU off, etc.) could fail silently. 

**Fix: one line** (`src/lidar/lidar.py:19`).

### 8.2 Non-functional command-line options

`--save False` and `--dark-mode False` have **no effect**: a classic argparse pitfall (`type=bool` converts any non-empty string to `True`). Replace with `store_true`/`store_false` flags.

### 8.3 Motion correction disabled

See §5.5: sensor in motion = distorted point cloud. Correct if necessary and re-enable.

### 8.4 Processing capacity

The entire session is stored in RAM. For long sessions, it may be necessary to adjust the memory allocation.

### 8.5 GPIO

The `stop_event` created in `main.py` is never passed to the GPIO process, so it is possible that this has no effect (dead code).

### 8.6 Detection settings

The detection settings are hard-coded:

- 5 cm grid
- max. 10 walls
- threshold 100 points
- deduplication 50 cm

For more in-depth testing, these should be passed as arguments.

## 8.7 Output

The output files (points.npy) are raw point clouds. It would be useful to save the data once it has been processed. It would be even more useful to output the LiDAR’s position in the room as a function of time (keyframes).
