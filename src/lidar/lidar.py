import socket
import struct

class Lidar:
    def __init__(self, ip_lidar="192.168.1.62", ip_local="192.168.1.100", port_lidar=6101, port_local=6201):
        self.lidar_ip = ip_lidar
        self.local_ip = ip_local
        self.sending_port = port_lidar
        self.receiving_port = port_local

        self.sock = None

        self.lidar_format = (
            "<"
            + "4sII"  # FrameHeader
            + "IIII"  # DataInfo (includes TimeStamp)
            + "IIfffffff"  # LidarInsideState
            + "ffffffff"  # LidarCalibParam
            + "ffffffffI300H300B"  # Line Info + ranges[300] + intensities[300]
            + "II2B2B"  # FrameTail
        )
        self.lidar_size = struct.calcsize(self.lidar_format)

        self.imu_format = (
            "<"
            + "4sII"  # FrameHeader
            + "IIII"  # DataInfo (includes TimeStamp)
            + "4f3f3f"  # Quaternion[4] + angles[3] + accelerations[3]
            + "II2B2B"  # FrameTail
        )
        self.imu_size = struct.calcsize(self.imu_format)

        print("[Init] Variables set")

    def start_connection(self):
        """Binds the UDP socket to listen for incoming LiDAR data"""
        # For receiver
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.local_ip, self.receiving_port))
        print(f"[Connection] Listening to data on {self.local_ip}:{self.receiving_port}")

    def _send_command(self, payload: bytes):
        """Helper to send raw command structures over the socket"""
        try:
            self.sock.sendto(payload, (self.lidar_ip, self.sending_port))
        except Exception as e:
            print(f"[Error] Failed to transmit command: {e}")

    def start_lidar(self):
        """
        Commands the internal motor to spin up and begin laser acquisition.
        Equivalent to triggering the wake/start routines
        """
        # Header + Length + Command ID (Start) + Checksum
        # Standard Unitree SDK execution frame example:
        #cmd_start = bytes([0xAA, 0x55, 0x01, 0x01, 0x00, 0x02]) 
        #self._send_command(cmd_start)

        header = bytes([0xAA, 0x55])
        length = 0x01
        cmd_id = 0x01
        payload = bytes([0x00]) # standard sub-mode/reserve byte
        
        packet = header + bytes([length, cmd_id]) + payload
        checksum = sum(packet[2:]) & 0xFF
        packet += bytes([checksum])

        self._send_command(packet)
        print("[Init] Starting LiDAR")

    def stop_lidar(self):
        """
        Commands the LiDAR to enter low-power standby mode and spin down
        """
        #cmd_stop = bytes([0xAA, 0x55, 0x01, 0x02, 0x00, 0x03])
        #self._send_command(cmd_stop)
        
        header = bytes([0xAA, 0x55])
        length = 0x01
        cmd_id = 0x02
        payload = bytes([0x00]) # standard sub-mode/reserve byte
        
        packet = header + bytes([length, cmd_id]) + payload
        checksum = sum(packet[2:]) & 0xFF
        packet += bytes([checksum])

        self._send_command(packet)
        print("[Closing] Stopping LiDAR")

    def update_workmode(self, mode_mask: int):
        """
        Packs the 32-bit (uint32) workmode flag into an explicit control packet
        """
        header = bytes([0xAA, 0x55]) # Standard Unitree Protocol Sync Bytes
        cmd_id = 0x0A                # Command ID for Workmode changes
        length = 0x04                # 4 bytes for uint32
        
        # Pack the mask as a Little-Endian 32-bit Unsigned Integer ('<I')
        payload = struct.pack('<I', mode_mask)
        
        # Assemble frame
        packet = header + bytes([length, cmd_id]) + payload
        
        # Basic LRC / Checksum calculation (Summing payload bytes)
        checksum = sum(packet[2:]) & 0xFF
        packet += bytes([checksum])
        
        self._send_command(packet)
        print(f"[Init] Sent Workmode Mask: {mode_mask} (Hex: {hex(mode_mask)})")

    def parse_packet(self, raw_bytes):
        """Parses raw binary packet data into a readable Python dictionary"""
        if len(raw_bytes) == self.lidar_size:
            unpacked = struct.unpack(self.lidar_format, raw_bytes)

            packet = {
                "header": {
                    "header": unpacked[0],
                    "packet_type": unpacked[1],
                    "packet_size": unpacked[2],
                },
                "info": {
                    "seq": unpacked[3],
                    "payload_size": unpacked[4],
                    "timestamp_sec": unpacked[5],
                    "timestamp_nsec": unpacked[6],
                },
                "state": {
                    "sys_rotation_period": unpacked[7],
                    "com_rotation_period": unpacked[8],
                    "dirty_index": unpacked[9],
                    "packet_lost_up": unpacked[10],
                    "packet_lost_down": unpacked[11],
                    "apd_temperature": unpacked[12],
                    "apd_voltage": unpacked[13],
                    "laser_voltage": unpacked[14],
                    "imu_temperature": unpacked[15],
                },
                "param": {
                    "a_axis_dist": unpacked[16],
                    "b_axis_dist": unpacked[17],
                    "theta_angle_bias": unpacked[18],
                    "alpha_angle_bias": unpacked[19],
                    "beta_angle": unpacked[20],
                    "xi_angle": unpacked[21],
                    "range_bias": unpacked[22],
                    "range_scale": unpacked[23],
                },
                # Extraction lines slice the trailing arrays smoothly
                "line_info": {
                    "com_horizontal_angle_start": unpacked[24],
                    "com_horizontal_angle_step": unpacked[25],
                    "scan_period": unpacked[26],
                    "range_min": unpacked[27],
                    "range_max": unpacked[28],
                    "angle_min": unpacked[29],
                    "angle_increment": unpacked[30],
                    "time_increment": unpacked[31],
                    "point_num": unpacked[32],
                    # 300 points from index 33 to 333
                    "ranges": list(unpacked[33:333]),
                    # 300 points from index 333 to 633
                    "intensities": list(unpacked[333:633]),
                },
                "tail": {
                    "crc32": unpacked[633],
                    "msg_type_check": unpacked[634],
                    "reserve": unpacked[635:637],
                    "tail": unpacked[637:639],
                },
            }

        elif len(raw_bytes) == self.imu_size:
            unpacked = struct.unpack(self.imu_format, raw_bytes)

            packet = {
                "header": {
                    "header": unpacked[0],
                    "packet_type": unpacked[1],
                    "packet_size": unpacked[2],
                },
                "info": {
                    "seq": unpacked[3],
                    "payload_size": unpacked[4],
                    "timestamp_sec": unpacked[5],
                    "timestamp_nsec": unpacked[6],
                },
                "data": {
                    "quaternion": list(unpacked[7:11]),
                    "angular_velocity": list(unpacked[11:14]),
                    "linear_acceleration": list(unpacked[14:17]),
                },
                "tail": {
                    "crc32": unpacked[17],
                    "msg_type_check": unpacked[18],
                    "reserve": unpacked[19:21],
                    "tail": unpacked[21:23],
                },
            }

        else:
            print(f"[Error] Packet invalide, impossible to parse: {len(raw_bytes)}")
            packet = None

        return packet

    def receive_stream(self):
        """Infinite loop processing data frames in real-time"""
        try:
            while True:
                data, _ = self.sock.recvfrom(8192) 
                parsed_frame = self.parse_packet(data)
                
                if parsed_frame:
                    yield parsed_frame
        except KeyboardInterrupt:
            print("\n[Closing] Stopping Parser")
    
    def close(self):
        self.sock.close()
