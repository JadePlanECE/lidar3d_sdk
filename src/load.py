from pathlib import Path
import numpy as np

DATA_DIR = Path("./data")

def load_data(file_name=""):
    paths = {
        "points": DATA_DIR / f"points{file_name}.npy",
        "imu": DATA_DIR / f"imu{file_name}.npy",
    }

    for path in paths.values():
        if not path.exists():
            raise FileNotFoundError(path)

    pts = np.load(paths["points"], allow_pickle=True)[:-1]
    imu = np.load(paths["imu"], allow_pickle=True)[:-1]

    print(f"[Load] Loaded {len(pts):,} rows from {paths['points']}")
    print(f"[Load] Loaded {len(imu):,} rows from {paths['imu']}")

    return pts, imu
