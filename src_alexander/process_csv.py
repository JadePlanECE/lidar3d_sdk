import numpy as np
from scipy.spatial.transform import Rotation as R
from skimage.transform import hough_line, hough_line_peaks
import warnings

class Process:
    def __init__(self):
        pass

    # --- Preprocess ---
    def create_roll_pitch_yaw(self, df):
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

    def angle_lidar(self, df, angle=30.0):
        """
        Correct LiDAR points acquired with a tilt angle (degrees) around the y-axis, with indirect/clockwise rotation

        The dataframe has the following columns:
        x, y, z, intensity, time
        """
        theta = np.radians(-angle)

        x = df["x"].to_numpy()
        z = df["z"].to_numpy()

        df["x"] = x * np.cos(theta) - z * np.sin(theta)
        df["z"] = x * np.sin(theta) + z * np.cos(theta)

        return df

    def size_limits(self, df, threshold=30):
        return df[
            (df["x"] < threshold) &
            (df["y"] < threshold) &
            (df["z"] < threshold) &
            (df["x"] > -threshold) &
            (df["y"] > -threshold) &
            (df["z"] > -threshold)
        ].copy()

    def calculate_time(self, df):
        min_time = df['time'].drop_duplicates().nsmallest(2).iloc[-1]
        max_time = df['time'].drop_duplicates().nlargest(2).iloc[-1]
        #min_time = df['time'].min()
        #max_time = df['time'].max()
        min_x = df['x'].min()
        min_y = df['y'].min()
        max_x = df['x'].max()
        max_y = df['y'].max()
        return min_time, max_time, min_x, max_x, min_y, max_y

    # --- Process ---
    def deskew_points(self, df_pts, df_imu):
        """
        Vectorized IMU deskew
        """
        if df_pts.empty or df_imu.empty:
            return df_pts

        t0 = df_pts['time'].min()
        dt = (df_pts['time'] - t0).values  # shape (N,)

        # Build a unified time axis (seconds)
        imu_t = df_imu['time_sec'].values + df_imu['time_nsec'].values / 1e9

        # Interpolate yaw rate and accelerations onto each LiDAR point's timestamp
        gyro_x  = np.interp(df_pts['time'].values, imu_t, df_imu['roll'].values)
        gyro_y  = np.interp(df_pts['time'].values, imu_t, df_imu['pitch'].values)
        gyro_z  = np.interp(df_pts['time'].values, imu_t, df_imu['yaw'].values)

        acc_x   = np.interp(df_pts['time'].values, imu_t, df_imu['acc_x'].values)
        acc_y   = np.interp(df_pts['time'].values, imu_t, df_imu['acc_y'].values)
        acc_z   = np.interp(df_pts['time'].values, imu_t, df_imu['acc_z'].values)

        # Calculate angular displacement (small angle approximation)
        d_roll  = gyro_x * dt
        d_pitch = gyro_y * dt
        d_yaw   = gyro_z * dt

        # Calculate linear displacement approximation from acceleration
        dx = 0.5 * acc_x * (dt ** 2)
        dy = 0.5 * acc_y * (dt ** 2)
        dz = 0.5 * acc_z * (dt ** 2)

        x = df_pts['x'].values
        y = df_pts['y'].values
        z = df_pts['z'].values

        df_out = df_pts.copy()
        
        # Apply the inverse rotation and translation to transform points back to t0
        df_out['x'] = x - (d_yaw * y) + (d_pitch * z) - dx
        df_out['y'] = y * (d_yaw * x) - (d_roll * z) - dy
        df_out['z'] = z - (d_pitch * x) + (d_roll * y) - dz

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
