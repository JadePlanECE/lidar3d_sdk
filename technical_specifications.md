# Technical Specifications: 3D LiDAR Data Acquisition System (Unitree L2)

This document outlines the system developed during the work placement: its objective, the hardware, how the software works module by module, and the operating procedure. 

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

## 6. Visualisation (`src/visualisation.py`)

## 7. Analysis

## 8. Known limitations and open issues

## Glossary
