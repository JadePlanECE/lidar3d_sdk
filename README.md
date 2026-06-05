# Unitree 4D LiDAR L2

The LiDAR launch code is based on the SDK [link here](https://github.com/unitreerobotics/unilidar_sdk2/tree/main/unitree_lidar_sdk).

To establish the UDP connection (via Ethernet cable) between the LiDAR and the computer or Jetson you are using, please read the configuration instructions. You will then be provided with instructions on how to launch it.

After that, you can create a Python virtual environment to begin data processing.

Do not use TTL UART connection, we only use UDP. Also plug in the 12V AC/DC adapter to power on the LiDAR.

The files I work on most often:
- read_lidar.py
- include/example.h
- examples/example_lidar_udp.cpp

Structure of the code:
```
├── bin
│   ├── example_lidar_udp
│   ├── set_ip_address
│   ├── set_to_serial_mode
│   └── set_to_udp_mode
├── build
│   ├── CMakeCache.txt
│   ├── CMakeFiles
│   ├── cmake_install.cmake
│   └── Makefile
├── CMakeLists.txt
├── data
│   ├── imu.npy
│   ├── imu.csv
│   ├── points.csv
│   ├── points.npy
├── examples
│   ├── example_lidar_udp.cpp
│   └── set_ip_address.cpp
├── include
│   ├── example.h
│   ├── unitree_lidar_protocol.h
│   ├── unitree_lidar_sdk_config.h
│   ├── unitree_lidar_sdk.h
│   └── unitree_lidar_utilities.h
├── lib
│   └── ...
├── lidar_sdk
│   └── ...
├── read_lidar.py
├── README.md
├── requirements.txt
├── src
│   ├── lidar
│   ├── load.py
│   ├── main.py
│   ├── process.py
│   └── visualisation.py
```
18 directories, 59 files

## Configuration of LiDAR

To configure the LiDAR, you'll need to connect it to your computer via Ethernet.

Run the command in a terminal; the goal is to find the name of the Ethernet connection (e.g. enP8p1s0). You will also find IP addressess (inet 192.168.x.x netmask 255.255.255.0).
```
ifconfig
```

This command will help you find the name of the connection (e.g. Wired connection 1).
```
nmcli connection show
```

Run the commande to check if teh LiDAR is indeed sending data. Do `Ctrl C` to stop (you can also check the IP addresses here).
```
sudo tcpdump -i enP8p1s0 -n
```

This command allows you to manually configure the network interface to ensure that the LiDAR and the machine are on the same network (this  configuration is often temporary, and you may need to reconfigure it after a reboot).
```
sudo ifconfig enP8p1s0 192.168.1.100 netmask 255.255.255.0 up
```

This command allows you to manually change the static IP address and the subnet mask.
```
sudo nmcli connection modify "Wired connection 1" ipv4.addresses 192.168.1.100/24 ipv4.method manual
```

After that, we need to reboot the network interface.
```
sudo nmcli connection up "Wired connection 1"
```

Then we check the network, and the connection with the LiDAR.
```
ip route
ping 192.168.1.62
```

## Compilation and run of C++ code

Compilation
```
mkdir build

cd build

cmake .. && make -j2
```

Run
```
../bin/example_lidar_udp
```

## Python virtual environment

You need to quit the build folder for the following part (`cd ..`).

Install Python on your system if you haven't already.
```
apt install python3.10-venv
```

Create a virtual environment (rename the envirnment as you want).
```
python3 -m venv lidar_sdk
```

Enter the environment.
```
source lidar_sdk/bin/activate

```

Install pip (if not already here).
```
sudo apt-get install python3-pip
```

Then install all the required libraries.
```
pip install -r requirements.txt
```


**Warning:** To desactivate the virtual environment, run the command.
```
deactivate
```


**Warning:** To destroy the environment.
```
rm -rf lidar_sdk
```

## Data processing

Run the command to run the data processing.
```
python read_lidar.py
```

You can also add arguments:
- Maximum of points render (default = 200 000) `--max-pts`
- Port (default = 8050) `--port`
