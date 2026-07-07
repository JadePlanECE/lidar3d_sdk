import socket
import struct
import time

# Constants
TIME_OUT = 16.0
REGISTER_TIME = 120 # 2 minuts

# Frame delimiters
HEADER_SIZE   = 12
TAIL_SIZE     = 12
_FRAME_HEADER = bytes([0x55, 0xAA, 0x05, 0x0A])
_FRAME_TAIL   = bytes([0x00, 0xFF])

# Packet type
LIDAR_USER_CMD_PACKET_TYPE         = 100  # host → lidar : commande utilisateur
LIDAR_ACK_DATA_PACKET_TYPE         = 101  # lidar → host : accusé de réception
LIDAR_POINT_DATA_PACKET_TYPE       = 102  # lidar → host : scan 3D
LIDAR_IMU_DATA_PACKET_TYPE         = 104  # lidar → host : IMU
LIDAR_WORK_MODE_CONFIG_PACKET_TYPE = 107  # host → lidar : config work mode

# Command types
USER_CMD_RESET_TYPE   = 1
USER_CMD_STANDBY_TYPE = 2  # cmd_value 0 = start, 1 = standby

# Structure parser
_FMT_POINT_DATA = "<" + "4sII" + "IIII" + "IIfffffff" + "ffffffff" + "ffffffffI300H300B" + "II2B2B"
_FMT_IMU_DATA   = "<" + "4sII" + "IIII" + "4f3f3f" + "II2B2B"
_FMT_ACK_DATA   = "<" + "4sII" + "IIII" + "II2B2B"

# Size packets
PKT_SIZE_POINT = struct.calcsize(_FMT_POINT_DATA) # 1044 bytes
PKT_SIZE_IMU   = struct.calcsize(_FMT_IMU_DATA)   # 80 bytes
PKT_SIZE_ACK   = struct.calcsize(_FMT_ACK_DATA)   # 40 bytes


def _crc32(data: bytes) -> int:
    """CRC-32 as implemented in unitree_lidar_utilities.h"""
    crc = 0xFFFFFFFF
    for buffer in data:
        crc ^= buffer
        for _ in range(8):
            crc = ((crc >> 1) ^ 0xEDB88320) if (crc & 1) else (crc >> 1)
            crc &= 0xFFFFFFFF
    return (~crc) & 0xFFFFFFFF

def _build_frame(packet_type: int, payload: bytes) -> bytes:
    """
    Build a complete Unitree LiDAR frame
    """
    packet_size = HEADER_SIZE + len(payload) + TAIL_SIZE
    header = _FRAME_HEADER + struct.pack('<II', packet_type, packet_size)

    crc = _crc32(payload)
    tail = struct.pack('<II', crc, 0) + bytes([0x00, 0x00]) + _FRAME_TAIL

    return header + payload + tail


class Lidar:
    def __init__(self, ip_lidar="192.168.1.62", ip_local="192.168.1.100", port_lidar=6101, port_local=6201):
        self.lidar_ip = ip_lidar
        self.local_ip = ip_local
        self.sending_port = port_lidar
        self.receiving_port = port_local

        self.sock: socket.socket | None = None

        print("[Init] LiDAR object created and set")

    def start_connection(self):
        """Creates the UDP socket"""
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((self.local_ip, self.receiving_port))
        self.sock.settimeout(TIME_OUT)
        print(f"[Connection] Socket bound to {self.local_ip}:{self.receiving_port}")

    def close(self):
        if self.sock:
            self.sock.close()
            self.sock = None

    def _send_command(self, payload: bytes):
        """Send a raw frame to the LiDAR and wait for its ACK"""
        if self.sock is None:
            print("[Error] Socket not initialised - call start_connection() first")
            return
        try:
            sent = self.sock.sendto(payload, (self.lidar_ip, self.sending_port))
            #print(f"[Debug] Sent {sent} bytes to {self.lidar_ip}:{self.sending_port}")
        except Exception as e:
            print(f"[Error] sendto failed: {e}")
            return

    def start_lidar(self):
        """
        Start LiDAR rotation / laser acquisition
        """
        payload = struct.pack('<II', USER_CMD_STANDBY_TYPE, 0)
        frame   = _build_frame(LIDAR_USER_CMD_PACKET_TYPE, payload)
        self._send_command(frame)
        print("[Init] Starting LiDAR")
        time.sleep(TIME_OUT)

    def stop_lidar(self):
        """
        Put LiDAR into standby (spin down / low-power)
        """
        payload = struct.pack('<II', USER_CMD_STANDBY_TYPE, 1)
        frame   = _build_frame(LIDAR_USER_CMD_PACKET_TYPE, payload)
        self._send_command(frame)
        print("[Closing] Stopping LiDAR")

    def reset_lidar(self):
        """
        Reset the LiDAR
        """
        payload = struct.pack('<II', USER_CMD_RESET_TYPE, 0)
        frame   = _build_frame(LIDAR_USER_CMD_PACKET_TYPE, payload)
        print("[CMD] reset_lidar")
        self._send_command(frame)

    def update_workmode(self, mode_mask: int):
        """
        Set the LiDAR work-mode register
        """
        payload = struct.pack('<I', mode_mask)
        frame   = _build_frame(LIDAR_WORK_MODE_CONFIG_PACKET_TYPE, payload)
        self._send_command(frame)
        print(f"[Init] Sent Workmode: {mode_mask} = 0b{mode_mask:08b} ({hex(mode_mask)})")

    def parse_packet(self, raw_bytes) -> dict | None:
        """Parses raw binary packet data into a readable Python dictionary"""
        if len(raw_bytes) < HEADER_SIZE:
            return None

        magic, pkt_type, pkt_size = struct.unpack_from('<4sII', raw_bytes, 0)
        #print(f"[Debug] Type {pkt_type} | Size {pkt_size} | Len {len(raw_bytes)} | Magic {magic}") # --- to debug ONLY ---
        dispatch = {
            LIDAR_POINT_DATA_PACKET_TYPE: self._parse_point_data,
            LIDAR_IMU_DATA_PACKET_TYPE: self._parse_imu,
            LIDAR_ACK_DATA_PACKET_TYPE: self._parse_ack_data,
            #_: None
        }
        fn = dispatch.get(pkt_type)
        if fn:
            return fn(raw_bytes)
        else:
            print(f"[LiDAR Error] Packet type unknown: {pkt_type} ({len(raw_bytes)} bytes)")
            return None

    def _parse_ack_data(self, raw: bytes):
        """
        LidarAckDataPacket = 40 bytes
            FrameHeader (12): header[4] + packet_type(u32) + packet_size(u32)
            LidarAckData(16): packet_type(u32) + cmd_type(u32) + cmd_value(u32) + status(u32)
            FrameTail   (12): crc32(u32) + msg_type_check(u32) + reserve[2] + tail[2]
        """
        if len(raw) != PKT_SIZE_ACK:
            #print(f"[Error] Incorrect number of bytes for ACK: {len(raw)} bytes (expected {PKT_SIZE_ACK})")
            return

        unpacked = struct.unpack(_FMT_ACK_DATA, raw)

        match unpacked[6]:
            case 1: # sucess
                return
            case 2:
                print("[ACK] CRC error")
            case 3:
                print("[ACK] Header error")
            case 4:
                print("[ACK] Block error")
            case 5:
                print("[ACK] Wait error")
            case _:
                print(f"[ACK] Unwanted type: {unpacked[6]}")

    def _parse_point_data(self, raw: bytes) -> dict:
        if len(raw) != PKT_SIZE_POINT:
            #print(f"[Error] Incorrect number of bytes for LiDAR: {len(raw)} bytes (expected {PKT_SIZE_POINT})")
            return {}

        unpacked = struct.unpack(_FMT_POINT_DATA, raw)

        return {
            "header": {
                "header": unpacked[0],
                "packet_type": unpacked[1],
                "packet_size": unpacked[2]
            },
            "info": {
                "seq": unpacked[3],
                "payload_size": unpacked[4],
                "timestamp_sec": unpacked[5],
                "timestamp_nsec": unpacked[6]
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
                "imu_temperature": unpacked[15]
            },
            "param": {
                "a_axis_dist": unpacked[16],
                "b_axis_dist": unpacked[17],
                "theta_angle_bias": unpacked[18],
                "alpha_angle_bias": unpacked[19],
                "beta_angle": unpacked[20],
                "xi_angle": unpacked[21],
                "range_bias": unpacked[22],
                "range_scale": unpacked[23]
            },
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
                "ranges": list(unpacked[33:333]),
                "intensities": list(unpacked[333:633])
            },
            "tail": {
                "crc32": unpacked[633],
                "msg_type_check": unpacked[634],
                "reserve": unpacked[635:637],
                "tail": unpacked[637:639]
            }
        }

    def _parse_imu(self, raw: bytes) -> dict:
        if len(raw) != PKT_SIZE_IMU:
            #print(f"[Error] Incorrect number of bytes for IMU: {len(raw)} bytes (expected {PKT_SIZE_IMU})")
            return {}

        unpacked =  struct.unpack(_FMT_IMU_DATA, raw)

        return {
            "header": {
                "header": unpacked[0],
                "packet_type": unpacked[1],
                "packet_size": unpacked[2]
            },
            "info": {
                "seq": unpacked[3],
                "payload_size": unpacked[4],
                "timestamp_sec": unpacked[5],
                "timestamp_nsec": unpacked[6]
            },
            "data": {
                "quaternion": list(unpacked[7:11]),
                "angular_velocity": list(unpacked[11:14]),
                "linear_acceleration": list(unpacked[14:17])
            },
            "tail": {
                "crc32": unpacked[17],
                "msg_type_check": unpacked[18],
                "reserve": unpacked[19:21],
                "tail": unpacked[21:23]
            }
        }

    def receive_stream(self):
        """Infinite loop processing data frames in real-time"""
        start = time.time()
        """pts_time = time.time()
        imu_time = time.time()
        pts = []
        imu = []"""
        try:
            while True:
                try:
                    data, _ = self.sock.recvfrom(8192)
                    parsed_frame = self.parse_packet(data)

                    if parsed_frame:
                        yield parsed_frame
                        """if parsed_frame['header']['packet_type'] == LIDAR_POINT_DATA_PACKET_TYPE:
                            pts.append(time.time() - pts_time)
                            pts_time = time.time()
                        if parsed_frame['header']['packet_type'] == LIDAR_IMU_DATA_PACKET_TYPE:
                            imu.append(time.time() - imu_time)
                            imu_time = time.time()"""
                    
                    if time.time() - start > REGISTER_TIME:
                        break # go out of the while loop
                except socket.timeout:
                    pass
        except KeyboardInterrupt:
            print("\n[Closing] Stopping Parser")
            """mean_pts = sum(pts) / len(pts)
            mean_imu = sum(imu) / len(imu)
            print(f"[Process] Mean time of detecting lidar points {mean_pts}")
            print(f"[Process] Mean time of detecting imu points {mean_imu}")
            variance_pts = math.sqrt(sum((x - mean_pts) ** 2 for x in pts) / len(pts))
            variance_imu = math.sqrt(sum((x - mean_imu) ** 2 for x in imu) / len(imu))
            print(f"[Process] Variance of lidar points {variance_pts}")
            print(f"[Process] Variance of imu points {variance_imu}")"""
