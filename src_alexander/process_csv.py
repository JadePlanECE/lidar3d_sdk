import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.spatial import cKDTree
from sklearn.decomposition import PCA
from sklearn.cluster import DBSCAN
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

    def size_limits(self, df, x=20, y=3, z=2):
        return df[
            (df["x"].between(-x, x)) &
            (df["y"].between(-y, y)) &
            (df["z"].between(-z, z))
        ].copy()

    def calculate_time(self, df):
        #min_time = df['time'].drop_duplicates().nsmallest(2).iloc[-1]
        #max_time = df['time'].drop_duplicates().nlargest(2).iloc[-1]
        min_time = df['time'].min()
        max_time = df['time'].max()
        min_x = df['x'].min()
        min_y = df['y'].min()
        max_x = df['x'].max()
        max_y = df['y'].max()
        return min_time, max_time, min_x, max_x, min_y, max_y

    # --- Process ---
    def detect_floor_ceiling_ransac(self, df, n_iter=100, inlier_thresh=0.01, min_inliers=50):
        """
        Fit horizontal planes (floor / ceiling) using RANSAC, without relying
        on point density. Returns (z_floor, z_ceiling) and a DataFrame with
        floor and ceiling points removed.

        Works even when the ceiling LiDAR data has a hole or is sparse.
        """
        if df.empty:
            warnings.warn("[detect_floor_ceiling_ransac] df is empty")
            return df, None, None

        pts = df[['x', 'y', 'z']].values
        best_planes = []  # list of (z_value, inlier_count, is_horizontal)

        for _ in range(n_iter):
            # Sample 3 random points
            idx = np.random.choice(len(pts), 3, replace=False)
            p1, p2, p3 = pts[idx]

            # Plane normal via cross product
            v1 = p2 - p1
            v2 = p3 - p1
            normal = np.cross(v1, v2)
            norm_len = np.linalg.norm(normal)
            if norm_len < 1e-8:
                continue
            normal = normal / norm_len

            # Only keep near-horizontal planes (|nz| > 0.9 means < ~25° tilt)
            if abs(normal[2]) < 0.9:
                continue

            # Distance from all points to this plane
            d = normal[0] * p1[0] + normal[1] * p1[1] + normal[2] * p1[2]
            dist = np.abs(pts[:, 0] * normal[0] +
                          pts[:, 1] * normal[1] +
                          pts[:, 2] * normal[2] - d)
            inliers = np.where(dist < inlier_thresh)[0]

            if len(inliers) >= min_inliers:
                z_plane = np.mean(pts[inliers, 2])
                best_planes.append((z_plane, len(inliers)))

        if not best_planes:
            warnings.warn("[detect_floor_ceiling_ransac] No horizontal planes found, "
                          "falling back to percentile method")
            z_ceiling = float(df['z'].quantile(0.98))
        else:
            # Sort by Z: lowest = floor, highest = ceiling
            best_planes.sort(key=lambda x: x[0])
            z_ceiling = best_planes[-1][0]

        # Remove floor and ceiling bands
        margin = 0.05  # meters
        df_clean = df[
            (df['z'] < z_ceiling - margin)
        ].copy()

        print(f"[RANSAC] Ceiling at Z={z_ceiling:.2f}, {len(df_clean):,} pts remaining")
        return df_clean, 0.0, z_ceiling

    def extract_wall_strip_adaptive(self, df, z_floor, z_ceiling, n_strips=5, best_n=1):
        """
        Instead of a fixed mid-height strip, evaluate N evenly-spaced strips
        and return the one with the most points. This adapts to both train
        tubes (where points are concentrated on the sides) and rectangular
        rooms. Set best_n > 1 to return the union of the top N strips.
        """
        if df.empty:
            warnings.warn("[extract_wall_strip_adaptive] df is empty")
            return df

        height = z_ceiling - z_floor
        strip_h = height / (n_strips + 1)

        candidates = []
        for i in range(1, n_strips + 1):
            z_lo = z_floor + (i - 0.5) * strip_h
            z_hi = z_floor + (i + 0.5) * strip_h
            sub = df[df['z'].between(z_lo, z_hi)]
            candidates.append((len(sub), z_lo, z_hi, sub))

        candidates.sort(key=lambda x: x[0], reverse=True)

        if best_n == 1:
            _, z_lo, z_hi, best = candidates[0]
            print(f"[adaptive strip] Best strip Z=[{z_lo:.2f}, {z_hi:.2f}], "
                  f"{len(best):,} pts")
            return best.copy()
        else:
            import pandas as pd
            parts = [c[3] for c in candidates[:best_n]]
            merged = pd.concat(parts).drop_duplicates()
            print(f"[adaptive strip] Top-{best_n} strips merged: {len(merged):,} pts")
            return merged.copy()

    def estimate_normals(self, df, k=20):
        """
        Estimate surface normals at each point using PCA on its K nearest
        neighbours. Returns the df with added columns nx, ny, nz.

        This is the LiDAR-agnostic key step: normals are purely geometric
        and work on any room shape.
        """
        if df.empty or len(df) < k + 1:
            warnings.warn("[estimate_normals] Not enough points")
            df['nx'] = 0.0
            df['ny'] = 0.0
            df['nz'] = 0.0
            return df

        pts = df[['x', 'y', 'z']].values
        tree = cKDTree(pts)
        normals = np.zeros_like(pts)

        for i, p in enumerate(pts):
            _, idx = tree.query(p, k=k + 1)
            neighbours = pts[idx[1:]]  # exclude self
            pca = PCA(n_components=3)
            pca.fit(neighbours - neighbours.mean(axis=0))
            # The smallest eigenvector (last component) is the normal
            normals[i] = pca.components_[-1]

        df = df.copy()
        df['nx'] = normals[:, 0]
        df['ny'] = normals[:, 1]
        df['nz'] = normals[:, 2]
        return df

    def segment_by_normals(self, df, eps=0.15, min_samples=30, use_xyz=True, xyz_weight=0.3):
        """
        Cluster points into surface patches based on normal direction similarity
        (+ optionally spatial proximity) using DBSCAN.

        Each cluster = one roughly planar (or smoothly curved) surface patch.
        Returns df with an added 'patch' column (-1 = noise).

        eps controls how similar normals must be (in normal-space units).
        xyz_weight scales the spatial component relative to normals.
        """
        if df.empty:
            warnings.warn("[segment_by_normals] df is empty")
            df['patch'] = -1
            return df

        if 'nx' not in df.columns:
            warnings.warn("[segment_by_normals] run estimate_normals() first")
            df['patch'] = -1
            return df

        normals = df[['nx', 'ny', 'nz']].values

        if use_xyz:
            pts = df[['x', 'y', 'z']].values
            # Normalise spatial coords to same scale as unit normals
            pts_norm = pts / (np.ptp(pts, axis=0).max() / xyz_weight + 1e-8)
            features = np.hstack([normals, pts_norm])
        else:
            features = normals

        db = DBSCAN(eps=eps, min_samples=min_samples, n_jobs=-1)
        labels = db.fit_predict(features)

        df = df.copy()
        df['patch'] = labels

        n_patches = len(set(labels)) - (1 if -1 in labels else 0)
        print(f"[segment_by_normals] {n_patches} surface patches found "
              f"({(labels == -1).sum():,} noise pts)")
        return df

    def find_corners_from_patches(self, df, min_patch_pts=80, corner_threshold=0.4, max_corners=20):
        """
        For each pair of adjacent surface patches, find their boundary region
        and fit a corner line (the intersection of the two best-fit planes).

        This replaces the Hough-based find_corners: it works on curved
        (tubular) rooms because it never assumes straight walls.

        Returns an (N, 3) array of corner XYZ positions, or None.
        """
        if df.empty or 'patch' not in df.columns:
            return None

        patches = [p for p in df['patch'].unique() if p != -1]
        if len(patches) < 2:
            warnings.warn("[find_corners_from_patches] Need at least 2 patches")
            return None

        # Fit a plane (centroid + normal) for each patch
        patch_planes = {}
        for pid in patches:
            sub = df[df['patch'] == pid][['x', 'y', 'z']].values
            if len(sub) < min_patch_pts:
                continue
            centroid = sub.mean(axis=0)
            pca = PCA(n_components=3).fit(sub - centroid)
            normal = pca.components_[-1]  # smallest variance = normal direction
            patch_planes[pid] = (centroid, normal)

        plane_ids = list(patch_planes.keys())
        if len(plane_ids) < 2:
            warnings.warn("[find_corners_from_patches] Not enough large patches")
            return None

        # Find boundary between each pair of adjacent patches using a KD-tree
        pts_all  = df[['x', 'y', 'z']].values
        labels   = df['patch'].values

        corners = []
        for i in range(len(plane_ids)):
            for j in range(i + 1, len(plane_ids)):
                pid_a = plane_ids[i]
                pid_b = plane_ids[j]

                pts_a = pts_all[labels == pid_a]
                pts_b = pts_all[labels == pid_b]

                # Check adjacency: nearest-neighbour distance between patches
                tree_b = cKDTree(pts_b)
                dists, _ = tree_b.query(pts_a, k=1)
                if dists.min() > 0.5:   # patches are more than 50 cm apart → skip
                    continue

                # ---- Intersect the two fitted planes ----
                c_a, n_a = patch_planes[pid_a]
                c_b, n_b = patch_planes[pid_b]

                # Intersection line direction = cross product of normals
                line_dir = np.cross(n_a, n_b)
                if np.linalg.norm(line_dir) < 1e-4:
                    continue  # nearly parallel planes
                line_dir /= np.linalg.norm(line_dir)

                # A point on the intersection line: solve the 2-plane system
                # n_a · (P - c_a) = 0  and  n_b · (P - c_b) = 0
                # We fix one coordinate to get a unique solution
                A = np.array([n_a, n_b, line_dir])
                b_vec = np.array([
                    np.dot(n_a, c_a),
                    np.dot(n_b, c_b),
                    np.dot(line_dir, (pts_a.mean(axis=0) + pts_b.mean(axis=0)) / 2)
                ])
                try:
                    pt_on_line = np.linalg.solve(A, b_vec)
                except np.linalg.LinAlgError:
                    continue

                # Project the boundary midpoint onto the line for a realistic position
                boundary_mid = np.vstack([
                    pts_a[np.argsort(dists)[:20]],
                    pts_b
                ]).mean(axis=0)
                corner_pt = pt_on_line + line_dir * np.dot(boundary_mid - pt_on_line,
                                                           line_dir)
                corners.append(corner_pt)

                if len(corners) >= max_corners:
                    break
            if len(corners) >= max_corners:
                break

        if not corners:
            return None

        # Deduplicate
        final = []
        for c in corners:
            if all(np.linalg.norm(c - e) >= corner_threshold for e in final):
                final.append(c)

        return np.array(final)

    def corners_process_adaptive(self, df, normal_k=20, dbscan_eps=0.15, dbscan_min=30, min_patch_pts=80, corner_threshold=0.4):
        """
        Full adaptive pipeline:
            1. RANSAC floor/ceiling removal
            2. Adaptive Z-strip selection
            3. Normal estimation
            4. Normal-based patch segmentation
            5. Corner detection from patch intersections

        Returns (corners, z_floor, z_ceiling) where corners is (N,3) or None.
        """
        if df.empty:
            print("[Error] corners_process_adaptive: empty DataFrame")
            return None, None, df

        # Step 1 – remove floor / ceiling
        df_clean, z_floor, z_ceiling = self.detect_floor_ceiling_ransac(df)

        if z_floor is None or df_clean.empty:
            print("[Error] RANSAC floor/ceiling detection failed")
            return None, None, df

        # Step 2 – adaptive Z-strip
        df_strip = self.extract_wall_strip_adaptive(df_clean, z_floor, z_ceiling)

        if df_strip.empty:
            print("[Error] No points in adaptive strip")
            return None, z_floor, df_clean

        # Downsample if too many points (normals are O(N*k))
        max_pts = 24000
        if len(df_strip) > max_pts:
            df_strip = df_strip.sample(n=max_pts, random_state=42)
            print(f"[adaptive] Downsampled to {max_pts} pts for normal estimation")

        # Step 3 – surface normals
        df_normals = self.estimate_normals(df_strip, k=normal_k)

        # Step 4 – patch segmentation
        df_patched = self.segment_by_normals(df_normals, eps=dbscan_eps, min_samples=dbscan_min)

        # Step 5 – corners from patch intersections
        corners = self.find_corners_from_patches(df_patched, min_patch_pts=min_patch_pts, corner_threshold=corner_threshold)

        if corners is not None:
            print(f"[adaptive] Found {len(corners)} corners via patch intersection")
            return corners, z_floor, df_clean

        print("[adaptive] No corners found")
        return None, z_floor, df_clean
