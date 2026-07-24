import numpy as np
from scipy.spatial.transform import Rotation as R
from skimage.transform import hough_line, hough_line_peaks
import warnings

class Process:
    def __init__(self):
        pass

    # --- Preprocess ---
    def convert_cloud(self, frame, range_min=0.0, range_max=100.0):
        """
        Converts a parsed packet into an (N, 5) NumPy array containing:
        [X, Y, Z, Intensity, Relative_Time] using optimized NumPy vectorization.
        """
        meta_data = frame[0]
        param = frame[1]
        line_info = frame[2]

        if line_info["point_num"] == 0:
            return []

        # 1. Convert core raw lists into NumPy arrays immediately
        raw_ranges = np.array(line_info["ranges"], dtype=np.float32)
        intensities = np.array(line_info["intensities"], dtype=np.float32)

        # 2. Vectorize the step-variable calculations (Alpha, Theta, Time)
        steps = np.arange(line_info["point_num"], dtype=np.float32)
        
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
            (range_float >= range_min) &
            (range_float <= range_max)
        )

        # If no points survive the filters, exit early
        if not np.any(valid_mask):
            return []

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

        return {
            'seq': meta_data["seq"],
            'timestamp_sec': meta_data["timestamp_sec"],
            'timestamp_nsec': meta_data["timestamp_nsec"],
            'timestamp_total': meta_data["timestamp_sec"] + meta_data["timestamp_nsec"] * 1e-9,

            'x': x,
            'y': y,
            'z': z,
            'intensity': intensities_filtered,
            'time': time_filtered,
        }

    def parse_imu(self, frame):
        meta_data = frame[0]
        sensor_data = frame[1]
        
        flat_entry = {
            "seq": meta_data["seq"],
            "time_sec": meta_data["timestamp_sec"],
            "time_nsec": meta_data["timestamp_nsec"],

            # Quaternions (Note: SciPy expects [x, y, z, w] by default)
            # Check if your data is [w, x, y, z] or [x, y, z, w]. 
            # Looking at your sample [0.998, 0.004, 0.001, 0.000], 0.998 is likely 'w' (the scalar).
            # SciPy's from_quat expects [x, y, z, w], so we map them accordingly:
            "qx": sensor_data["quaternion"][1],
            "qy": sensor_data["quaternion"][2],
            "qz": sensor_data["quaternion"][3],
            "qw": sensor_data["quaternion"][0], # Moving w to the end
            
            # Accelerations
            "acc_x": sensor_data["linear_acceleration"][0],
            "acc_y": sensor_data["linear_acceleration"][1],
            "acc_z": sensor_data["linear_acceleration"][2],
            
            # Angular Velocities (if you need them later)
            "gyro_x": sensor_data["angular_velocity"][0],
            "gyro_y": sensor_data["angular_velocity"][1],
            "gyro_z": sensor_data["angular_velocity"][2],
        }
        return flat_entry

    def size_limits(self, df, threshold=30):
        return df[
            (df["x"] < threshold) &
            (df["y"] < threshold) &
            (df["z"] < threshold) &
            (df["x"] > -threshold) &
            (df["y"] > -threshold) &
            (df["z"] > -threshold)
        ].copy()

    def create_roll_pitch_yaw(self, df):
        df["acc_z_corrected"] = df["acc_z"] - 9.81

        quats = df[["qx", "qy", "qz", "qw"]].to_numpy()
        quat_norms = np.linalg.norm(quats, axis=1)
        valid_mask = quat_norms > 1e-8

        eulers = np.zeros((len(df), 3))
        rotations = R.from_quat(quats[valid_mask])
        eulers[valid_mask] = rotations.as_euler('xyz', degrees=True)

        df["roll"]  = eulers[:, 0]
        df["pitch"] = eulers[:, 1]
        df["yaw"]   = eulers[:, 2]

        return df
    
    def calculate_time(self, df):
        #min_time = df['timestamp_total'].drop_duplicates().nsmallest(2).iloc[-1]
        #max_time = df['timestamp_total'].drop_duplicates().nlargest(2).iloc[-1]
        min_time = df['timestamp_total'].min()
        max_time = df['timestamp_total'].max()
        min_x = df['x'].min()
        min_y = df['y'].min()
        max_x = df['x'].max()
        max_y = df['y'].max()
        return min_time, max_time, min_x, max_x, min_y, max_y

    # --- Process ---
    def deskew_points(self, df_pts, df_imu):
        """
        Vectorized IMU deskew — corrects LiDAR motion distortion using IMU data
        """
        if df_pts.empty or df_imu.empty:
            print("[Warning] Datasets empty (deskew step)")
            return df_pts

        # Reference time: first point of the scan (in seconds)
        t0 = df_pts['time'].min()

        # Heuristic: if timestamps are > 1e12, they are likely in nanoseconds
        if t0 > 1e12:
            lidar_t_sec = df_pts['time'].values / 1e9
            t0_sec = t0 / 1e9
        else:
            lidar_t_sec = df_pts['time'].values
            t0_sec = t0

        dt = lidar_t_sec - t0_sec  # shape (N,), in seconds

        # Build IMU time axis in seconds
        imu_t = df_imu['time_sec'].values + df_imu['time_nsec'].values / 1e9

        # These should be the raw gyroscope readings from the IMU
        gyro_x = np.interp(lidar_t_sec, imu_t, df_imu['ang_x'].values)
        gyro_y = np.interp(lidar_t_sec, imu_t, df_imu['ang_y'].values)
        gyro_z = np.interp(lidar_t_sec, imu_t, df_imu['ang_z'].values)

        acc_x  = np.interp(lidar_t_sec, imu_t, df_imu['acc_x'].values)
        acc_y  = np.interp(lidar_t_sec, imu_t, df_imu['acc_y'].values)
        acc_z  = np.interp(lidar_t_sec, imu_t, df_imu['acc_z'].values)

        # Angular displacement (small angle approximation, radians)
        d_roll  = gyro_x * dt
        d_pitch = gyro_y * dt
        d_yaw   = gyro_z * dt

        # Linear displacement from double-integrating acceleration
        dx = 0.5 * acc_x * (dt ** 2)
        dy = 0.5 * acc_y * (dt ** 2)
        dz = 0.5 * acc_z * (dt ** 2)

        x = df_pts['x'].values
        y = df_pts['y'].values
        z = df_pts['z'].values

        df_out = df_pts.copy()

        # Inverse (undo motion → bring point back to t0 frame):
        df_out['x'] =           x + d_yaw  * y - d_pitch * z - dx
        df_out['y'] = - d_yaw * x +          y + d_roll  * z - dy
        df_out['z'] = d_pitch * x - d_roll * y +           z - dz

        return df_out

    def find_ceiling(self, df):
        """
        Finds the ceiling height by looking for the statistical mode of the Z-axis
        We must put more weight to the points closer to x=0 and y=0
        """
        if df.empty:
            warnings.warn("[WARNING: find_ceiling] Directory 'df' is empty")
            return 0
        
        df = df.copy()
        df['weight'] = 20 - (df['x'].abs() + df['y'].abs())
        df['weight'] = df['weight'].clip(lower=0.1)

        z_groups = df.groupby(df['z'].round(2))['weight'].sum()
        ceiling_z = z_groups.idxmax()
        
        #print(f"[Process] Detected ceiling at Z = {ceiling_z:.2f}")
        return ceiling_z

    def erease_ceiling(self, df, z, threshold=0.4):
        """
        Removes points that are within a threshold of the detected ceiling + the upper part
        """
        if df.empty:
            warnings.warn("[WARNING: erase_ceiling] Directory 'df' is empty")
            return df
    
        return df[df['z'] < (z - threshold)].copy()

    def find_walls(self, df, ceiling, margin=0.2):
        """
        Classifies points as walls if they are in the middle between :
        - the ceiling (erased)
        - 5% of the lowest points
        df is then a strip (in theory, the shape of the room)
        """
        if df.empty:
            warnings.warn("[WARNING: find_walls] Directory 'df' is empty")
            return df
        
        lowest_5 = df['z'].quantile(0.05)
        middle = (ceiling - lowest_5) / 2

        # Calcul des bornes
        lower = middle * (1 - margin)
        upper = middle * (1 + margin)

        strip = df[df['z'].between(lower, upper)]
        #print(f"[Process] Wall strip [{lower:.2f}, {upper:.2f}]: {len(strip):,} points")
        return strip

    def find_corners(self, df, max_walls=10, min_pts=100, corner_threshold=0.5):
        points = df[['x', 'y']].values
        if len(points) < min_pts:
            return None

        # Rasterize into a 2D grid
        res = 0.05  # 5 cm per cell
        x_min, y_min = points.min(axis=0) - 0.5
        x_max, y_max = points.max(axis=0) + 0.5
        
        W = int((x_max - x_min) / res) + 1
        H = int((y_max - y_min) / res) + 1
        grid = np.zeros((H, W), dtype=np.uint8)

        xi = ((points[:, 0] - x_min) / res).astype(int)
        yi = ((points[:, 1] - y_min) / res).astype(int)
        xi = np.clip(xi, 0, W - 1)
        yi = np.clip(yi, 0, H - 1)
        grid[yi, xi] = 1

        # Hough transform
        tested_angles = np.linspace(-np.pi / 2, np.pi / 2, 360, endpoint=False)
        h, theta, d = hough_line(grid, theta=tested_angles)
        
        # Extract dominant lines
        _, angles, dists = hough_line_peaks(h, theta, d, num_peaks=max_walls, min_distance=20, min_angle=10)

        # Convert lines back to (A, B, C) form: x*cos(t) + y*sin(t) = d
        walls_params = []
        for angle, dist in zip(angles, dists):
            # Real-world dist (account for grid offset)
            dist_rw = dist * res
            A = np.cos(angle)
            B = np.sin(angle)
            C = -(dist_rw + A * x_min + B * y_min)
            walls_params.append((A, B, C))

        # Intersect pairs → corners (same logic as your current code)
        corners = []
        for i in range(len(walls_params)):
            for j in range(i + 1, len(walls_params)):
                A1, B1, C1 = walls_params[i]
                A2, B2, C2 = walls_params[j]
                det = A1 * B2 - A2 * B1
                if abs(det) < 1e-3:
                    continue
                px = (B1 * C2 - B2 * C1) / det
                py = (A2 * C1 - A1 * C2) / det
                # Check corner is within point cloud bounds
                if x_min <= px <= x_max and y_min <= py <= y_max:
                    corners.append(np.array([px, py]))

        # Deduplicate
        final = []
        for c in corners:
            if all(np.linalg.norm(c - e) >= corner_threshold for e in final):
                final.append(c)

        return np.array(final) if final else None

    def ceiling_process(self, df):
        ceiling_z = self.find_ceiling(df)
        df_no_ceiling = self.erease_ceiling(df, ceiling_z)
        return df_no_ceiling, ceiling_z

    def corners_process(self, df, z):
        df_walls = self.find_walls(df, z)

        if df_walls.empty:
            print("[Error] No wall points found.")
            return None

        return self.find_corners(df_walls)
