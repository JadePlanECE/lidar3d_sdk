import argparse
import load_csv
import process_csv
import visu_csv

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=None)
    parser.add_argument("--file-name", type=str, default="-lidar3", help="Name of the CSV files stored in the 'data' folder")
    parser.add_argument("--file-part", type=str, default="1", help="Part of the LiDAR CSV files stored in the 'data' folder")
    parser.add_argument("--delta", type=float, default=0.1, help="Delta time to get data")
    parser.add_argument("--angle", type=float, default=30.0, help="Angle of LiDAR when running (in degrees)")
    parser.add_argument("--port", type=int, default=8050, help="Port for Dash visualisation")
    parser.add_argument("--max-pts", type=int, default=300000, help="Max point rows to visualize")
    parser.add_argument("--dark-mode", type=bool, default=True, help="Visalisation in dark mode")
    args = parser.parse_args()

    # Load data from CSV file
    df_pts = load_csv.load_data_points(args.file_name, args.file_part)
    df_imu = load_csv.load_data_imu(args.file_name)

    # Process data with numpy (points + IMU)
    processeur = process_csv.Process()
    df_imu = processeur.create_roll_pitch_yaw(df_imu)
    df_pts = processeur.angle_lidar(df_pts, args.angle) # rotate data according to the actual lidar angle
    df_pts = processeur.size_limits(df_pts) # set a distance limit to remove phantom or unprocessable points

    # Dash vizualisation
    display = visu_csv.Visualisation(df_pts=df_pts, df_imu=df_imu, port=args.port, max_pts=args.max_pts, darkmode=args.dark_mode, delta=args.delta)
    display.visualisation_data()
