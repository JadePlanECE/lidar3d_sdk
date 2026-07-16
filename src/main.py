import argparse
import pandas as pd
import multiprocessing as mp

import lidar.lidar_manager as lidar
import load
import gpio
import process
import visualisation as viz

def main(args:argparse.Namespace, checkup_lidar, checkup_viz):
    if args.file_name == None:
        # Receive data from liDAR (UDP)
        receiver = lidar.LidarManager(
            delta=args.delta,
            save=args.save,
            status=checkup_lidar
        )
        df_pts, df_imu = receiver.get_data()
    else:
        # Load data from CSV file
        df_pts, df_imu = load.load_data(file_name=args.file_name)

    # Process data with numpy (points + IMU)
    processeur = process.Process()
    df_pts_process = []
    df_imu_process = []
    for frame in df_pts:
        df_pts_process.append(processeur.convert_cloud(frame))
    for frame in df_imu:
        df_imu_process.append(processeur.parse_imu(frame))

    if (df_pts_process[0] is None) or (df_pts_process[0] == []) or (df_pts_process[0] == {}):
        df_pts_process.pop(0)

    df_pts_process = pd.DataFrame(df_pts_process)
    df_imu_process = pd.DataFrame(df_imu_process)

    if df_pts_process.empty or df_imu_process.empty:
        print("[Error] Dataframes empty")
        raise SystemExit from None

    cols = ["x", "y", "z", "intensity", "time"]
    df_pts_process = df_pts_process.explode(cols, ignore_index=True)
    df_pts_process[cols] = df_pts_process[cols].apply(pd.to_numeric, errors="coerce")

    df_pts_process = processeur.size_limits(df_pts_process, threshold=30)
    df_imu_process = processeur.create_roll_pitch_yaw(df_imu_process)

    # Dash vizualisation
    display = viz.Visualisation(
        df_pts=df_pts_process,
        df_imu=df_imu_process,
        port=args.port,
        max_pts=args.max_pts,
        darkmode=args.dark_mode,
        delta=args.delta,
        status=checkup_viz
    )
    display.visualisation_data()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=None)
    parser.add_argument("--file-name", type=str, default=None, help="Name of the CSV files stored in the 'data' folder")
    parser.add_argument("--delta", type=float, default=0.1, help="Delta time to get data")
    parser.add_argument("--save", type=bool, default=True, help="Save data in CSV files")
    parser.add_argument("--port", type=int, default=8050, help="Port for Dash visualisation")
    parser.add_argument("--max-pts", type=int, default=200000, help="Max point rows to visualize")
    parser.add_argument("--dark-mode", type=bool, default=True, help="Visalisation in dark mode")
    args = parser.parse_args()

    checkup_lidar = mp.Value('b', True) # Boolean
    checkup_viz = mp.Value('b', True) # Boolean

    # Start gpio traking
    pin_tracking = gpio.Pins()
    stop_event = mp.Event()
    tracking_process = mp.Process(
        target=pin_tracking.tracking,
        args=(checkup_lidar, checkup_viz),
        daemon=True
    )
    tracking_process.start()

    try:
        main(args, checkup_lidar, checkup_viz)
    except KeyboardInterrupt:
        print("\n[Main] Interrupted")
    finally:
        stop_event.set()
        tracking_process.join(timeout=2)
        if tracking_process.is_alive():
            tracking_process.terminate()
 