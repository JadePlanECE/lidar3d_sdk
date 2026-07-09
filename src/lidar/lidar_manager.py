import numpy as np
import time
import lidar.lidar as lidar

def generate_workmode_mask(wide_fov=False, mode_2d=False, disable_imu=False, serial_mode=False, standby_on_boot=False):
    mask = 0
    if wide_fov: mask |= (1 << 0)
    if mode_2d: mask |= (1 << 1)
    if disable_imu: mask |= (1 << 2)
    if serial_mode: mask |= (1 << 3)
    if standby_on_boot: mask |= (1 << 4)
    return mask

class LidarManager:
    def __init__(self, delta, save, status):
        self.delta = delta
        self.save = save
        self.status = status

    def get_data(self):
        parser = lidar.Lidar("192.168.1.62", "192.168.1.100", 6101, 6201, self.status)
        parser.start_connection()

        mask = generate_workmode_mask()
        parser.update_workmode(mask)

        parser.start_lidar()

        pts = []
        imu = []

        for frame in parser.receive_stream():
            match frame['header']['packet_type']:
                case 102:
                    pts.append([
                        frame['info'],
                        frame['param'],
                        frame['line_info']
                    ])
                case 104:
                    imu.append([
                        frame['info'],
                        frame['data']
                    ])

        points = np.asarray(pts, dtype=object)
        imu_data = np.asarray(imu, dtype=object)

        parser.stop_lidar()
        parser.close()

        if self.save:
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            np.save(f"./data/points_{timestamp}.npy", points)
            np.save(f"./data/imu_{timestamp}.npy", imu_data)

        return points, imu_data

if __name__ == "__main__":
    receiver = LidarManager(delta=100, save=True)
    df_pts, df_imu = file_name = receiver.get_data()
