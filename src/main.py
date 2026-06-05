import argparse
import numpy as np
import pandas as pd
import lidar.lidar_manager as lidar
import load
import process
import visualisation as viz

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=".")
    parser.add_argument("--file-name", type=str, default="", help="Name of the CSV files stored in the 'data' folder")
    parser.add_argument("--delta", type=int, default=100, help="Delta time to get data")
    parser.add_argument("--save", type=bool, default=True, help="Save data in CSV files")
    parser.add_argument("--port", type=int, default=8050, help="Port for Dash visualisation")
    parser.add_argument("--max-pts", type=int, default=200000, help="Max point rows to visualize")
    parser.add_argument("--dark-mode", type=bool, default=True, help="Visalisation in dark mode")
    args = parser.parse_args()

    if args.file_name == None:
        # Receive data from liDAR (UDP)
        receiver = lidar.LidarManager(delta=args.delta, save=args.save)
        df_pts, df_imu = file_name = receiver.get_data()
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

    df_pts_process = pd.DataFrame(df_pts_process)
    df_imu_process = pd.DataFrame(df_imu_process)
    df_pts_process = df_pts_process.explode(["x", "y", "z", "intensity", "time"], ignore_index=True)
    
    df_imu_process = processeur.create_roll_pitch_yaw(df_imu_process)

    # Dash vizualisation
    display = viz.Visualisation(df_pts=df_pts_process, df_imu=df_imu_process, port=args.port, max_pts=args.max_pts, darkmode=args.dark_mode, delta=args.delta)
    display.visualisation_data()
