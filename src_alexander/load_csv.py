import sys
from pathlib import Path
import pandas as pd

DATA_DIR = Path("./data/alexander")

def load_data_points(file_name, part_file):
    if part_file is None:
        path = DATA_DIR / f"points{file_name}.csv"
    else:
        path = DATA_DIR / f"points{file_name}/split_part_{part_file}.csv"

    if not path.exists():
        sys.exit(f"[Error] File not found {path}\n")

    df = pd.read_csv(path)
    df = df.iloc[:-1] #drop the last line (always unfinished)
    print(f"[Load] Loaded {len(df):,} rows from {path}")
    return df

def load_data_imu(file_name):
    path = DATA_DIR / f"imu{file_name}.csv"

    if not path.exists():
        sys.exit(f"[Error] File not found {path}\n")

    df = pd.read_csv(path)
    df = df.iloc[:-1] #drop the last line (always unfinished)
    print(f"[Load] Loaded {len(df):,} rows from {path}")
    return df
