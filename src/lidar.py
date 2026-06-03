import socket
import struct
import numpy as np

def generate_workmode_mask(wide_fov=False, mode_2d=False, disable_imu=False, serial_mode=False, standby_on_boot=False):
    mask = 0
    if wide_fov: mask |= (1 << 0)
    if mode_2d: mask |= (1 << 1)
    if disable_imu: mask |= (1 << 2)
    if serial_mode: mask |= (1 << 3)
    if standby_on_boot: mask |= (1 << 4)
    return mask

class LidarParser:
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
        print(f"[Connection] Listening for Unitree L2 LiDAR data on {self.local_ip}:{self.receiving_port}...")

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
        print("[Init] LiDAR starting")

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
        print("[closing] LiDAR stoping")

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

    def convert_to_point_cloud(self, parsed_frame, range_min_limit=0.0, range_max_limit=100.0):
        """
        Converts a parsed packet into an (N, 5) NumPy array containing:
        [X, Y, Z, Intensity, Relative_Time] using optimized NumPy vectorization.
        """
        param = parsed_frame["param"]
        line_info = parsed_frame["line_info"]
        num_of_points = line_info["point_num"]

        if num_of_points == 0:
            return np.empty((0, 5))

        # 1. Convert core raw lists into NumPy arrays immediately
        raw_ranges = np.array(line_info["ranges"], dtype=np.float64)
        intensities = np.array(line_info["intensities"], dtype=np.float64)

        # 2. Vectorize the step-variable calculations (Alpha, Theta, Time)
        steps = np.arange(num_of_points, dtype=np.float64)
        
        alpha_arr = (line_info["angle_min"] + param["alpha_angle_bias"]) + (steps * line_info["angle_increment"])
        theta_arr = (line_info["com_horizontal_angle_start"] + param["theta_angle_bias"]) + (steps * line_info["com_horizontal_angle_step"])
        time_arr = steps * line_info["time_increment"]

        # 3. Vectorize the Range Float conversion
        range_float = param["range_scale"] * (raw_ranges + param["range_bias"])

        # 4. Generate combined boolean mask for ALL 3 filters simultaneously
        valid_mask = (
            (raw_ranges >= 1) &
            (range_float >= line_info["range_min"]) &
            (range_float <= line_info["range_max"]) &
            (range_float >= range_min_limit) &
            (range_float <= range_max_limit)
        )

        # If no points survive the filters, exit early
        if not np.any(valid_mask):
            return np.empty((0, 5))

        # 5. Filter all arrays down to only the valid points
        range_filtered = range_float[valid_mask]
        alpha_filtered = alpha_arr[valid_mask]
        theta_filtered = theta_arr[valid_mask]
        intensities_filtered = intensities[valid_mask]
        time_filtered = time_arr[valid_mask]

        # 6. Pre-calculate Trigonometric constants
        beta = param["beta_angle"]
        xi = param["xi_angle"]
        
        sin_beta = np.sin(beta)
        cos_beta = np.cos(beta)
        sin_xi = np.sin(xi)
        cos_xi = np.cos(xi)

        cos_beta_sin_xi = cos_beta * sin_xi
        sin_beta_cos_xi = sin_beta * cos_xi
        sin_beta_sin_xi = sin_beta * sin_xi
        cos_beta_cos_xi = cos_beta * cos_xi

        # 7. Dynamic Trigonometric element-wise operations
        sin_alpha = np.sin(alpha_filtered)
        cos_alpha = np.cos(alpha_filtered)
        sin_theta = np.sin(theta_filtered)
        cos_theta = np.cos(theta_filtered)

        # 8. C++ Matrix math translations (fully vectorized)
        A = (-cos_beta_sin_xi + sin_beta_cos_xi * sin_alpha) * range_filtered + param["b_axis_dist"]
        B = cos_alpha * cos_xi * range_filtered
        C = (sin_beta_sin_xi + cos_beta_cos_xi * sin_alpha) * range_filtered

        x = cos_theta * A - sin_theta * B
        y = sin_theta * A + cos_theta * B
        z = C + param["a_axis_dist"]

        # 9. Stack columns horizontally to create the final (N, 5) shape
        return np.column_stack((x, y, z, intensities_filtered, time_filtered))

    def receive_stream(self):
        """Infinite loop processing data frames in real-time"""
        try:
            while True:
                # Buffer size typical for heavy LiDAR MTU sizes (usually 1500 or 8192)
                data, addr = self.sock.recvfrom(8192) 
                parsed_frame = self.parse_packet(data)
                
                if parsed_frame:
                    yield parsed_frame
        except KeyboardInterrupt:
            print("\n[Closing] Stopping LiDAR Parser")
        finally:
            parser.stop_lidar()
            self.sock.close()

if __name__ == "__main__":
    parser = LidarParser("192.168.1.62", "192.168.1.100", 6101, 6201)
    parser.start_connection()

    mask = generate_workmode_mask()
    parser.update_workmode(mask)

    parser.start_lidar()

    for frame in parser.receive_stream():
        packet_type = frame['header']['packet_type']
        packet_size = frame['header']['packet_size']

        match packet_type:
            case 102:
                packet_type = "LiDAR points"
                #pc = parser.convert_to_point_cloud(frame, range_min_limit=0.01, range_max_limit=60.0)
            case 104:
                packet_type = "IMU data    "
                #pc = None

        print(f"[Process] Packet from {packet_type} | size {packet_size}")
        #if pc and pc.shape[0] > 0:
        #    print(f"    First Point Sample -> X: {pc[0,0]:.3f}m, Y: {pc[0,1]:.3f}m, Z: {pc[0,2]:.3f}m")
