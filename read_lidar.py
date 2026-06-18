import argparse
import sys
import os
import warnings
import numpy as np
import pandas as pd
import plotly.express as px
import plotly.graph_objects as go
from plotly.subplots import make_subplots
from sklearn.linear_model import RANSACRegressor
from scipy.spatial.transform import Rotation as R
from skimage.transform import hough_line, hough_line_peaks
import dash
import dash_daq as daq

# Constants
NBR_POINTS = 200000
TRAIN_HEIGHT = 3.0
DELTA = 0.1
DARK_MODE = True

THEMES = {
    "light": {
        "bg_app": "#f5f7fb",
        "bg_card": "white",
        "text_main": "#1f2937",
        "text_sub": "#4b5563",
        "switch": "#3b82f6",
        "plotly_template": "plotly_white",
        "grid_color": "#e5e7eb",
        "border": "1px solid #e5e7eb",
        "marker_lidar": "black",
        "marker_corners": "green",
        "marker_dist": "black"
    },
    "dark": {
        "bg_app": "#111827",
        "bg_card": "#1f2937",
        "text_main": "#f9fafb",
        "text_sub": "#d1d5db",
        "switch": "#14b8a6",
        "plotly_template": "plotly_dark",
        "grid_color": "#374151",
        "border": "1px solid #374151",
        "marker_lidar": "#14b8a6",
        "marker_corners": "#036B62",
        "marker_dist": "white"
    }
}

COLOR = THEMES["dark"] if DARK_MODE else THEMES["light"]

# Paths
#FILES = ".csv"
#FILES = "_time.csv"
#FILES = "_huge.csv"
#FILES = "_interesting.csv"
#FILES = "_test.csv"
#FILES = "-lidar3.csv"
#FILES = "_20230315_153132-lidar2.csv"
FILES = "_20230315_152320-lidar1.csv"

#POINTS_CSV = "./data/points" + FILES
#IMU_CSV = "./data/imu" + FILES
#POINTS_CSV = "./data/alexander/points" + FILES
POINTS_CSV = "./data/alexander/split_files/split_part_3.csv"
IMU_CSV = "./data/alexander/imu" + FILES

# Load data
def load_csv(path):
    if not os.path.exists(path):
        sys.exit(f"[Error] File not found {path}\n")
    
    df = pd.read_csv(path)
    print(f"[Load CSV] Loaded {len(df):,} rows from {path}")
    df = df.iloc[:-1] #drop the last line (always unfinished)
    return df

def create_roll_pitch_yaw(df):
    df = df.copy()
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

# Process
def deskew_points(df_window, df_imu):
    """
    Vectorized IMU deskew — compensates platform motion within the time window.
    Reference pose = start of window (t0).
    """
    if df_window.empty or df_imu.empty:
        return df_window

    t0 = df_window['time'].min()
    dt = (df_window['time'] - t0).values  # shape (N,)

    # Build a unified time axis for IMU (seconds)
    imu_t = df_imu['time_sec'].values + df_imu['time_nsec'].values / 1e9

    # Interpolate yaw rate and accelerations onto each LiDAR point's timestamp
    # (use cumulative yaw, not yaw itself, for angular displacement)
    gyro_z  = np.interp(df_window['time'].values, imu_t, df_imu['yaw'].values)
    acc_x   = np.interp(df_window['time'].values, imu_t, df_imu['acc_x'].values)
    acc_y   = np.interp(df_window['time'].values, imu_t, df_imu['acc_y'].values)

    dtheta = gyro_z * dt
    dx     = 0.5 * acc_x * dt**2
    dy     = 0.5 * acc_y * dt**2

    cos_t = np.cos(-dtheta)
    sin_t = np.sin(-dtheta)

    x = df_window['x'].values
    y = df_window['y'].values

    df_out = df_window.copy()
    df_out['x'] = cos_t * (x - dx) - sin_t * (y - dy)
    df_out['y'] = sin_t * (x - dx) + cos_t * (y - dy)

    return df_out

def ceiling_process(df):
    ceiling_z = find_ceiling(df)
    #df_no_ceiling = erase_ceiling(df, ceiling_z)
    return df, ceiling_z

def corners_process(df, z):
    df_walls = find_walls(df, z)

    if df_walls.empty:
        print("[Error] No wall points found.")
        return None

    return find_corners_hough(df_walls) #find_corners(df_walls)

def find_ceiling(df):
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

def erase_ceiling(df, z, threshold=0.4):
    """
    Removes points that are within a threshold of the detected ceiling + the upper part
    """
    if df.empty:
        warnings.warn("[WARNING: erase_ceiling] Directory 'df' is empty")
        return df
    
    return df[df['z'] < (z - threshold)].copy()

def find_walls(df, ceiling, margin=0.2):
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

def find_corners(df, max_walls=10, min_points=100, dist_threshold=0.2, corner_threshold=0.5):
    """
    1. Detect wall lines using RANSAC
    2. Project points onto lines to find endpoints
    3. Intersect adjacent lines to find physical corners
    """
    points = df[['x', 'y']].values
    walls = []
    remaining_points = points.copy()

    for _ in range(max_walls):
        if len(remaining_points) < min_points:
            break

        if np.var(remaining_points[:,0]) > np.var(remaining_points[:,1]):
            x, y = remaining_points[:,0:1], remaining_points[:,1]
            is_vertical = False
        else:
            x, y = remaining_points[:,1:2], remaining_points[:,0]
            is_vertical = True

        ransac = RANSACRegressor(residual_threshold=0.05)
        try:
            ransac.fit(x,y)
        except: break

        inlier_mask = ransac.inlier_mask_
        if np.sum(inlier_mask) < min_points:
            break

        m = ransac.estimator_.coef_[0]
        b = ransac.estimator_.intercept_
        params = (m, -1, b) if not is_vertical else (1, -m, -b)

        walls.append({'params':params, 'pts':remaining_points[inlier_mask]})
        remaining_points = remaining_points[~inlier_mask]
    
    corners = []
    for i in range(len(walls)):
        for j in range(i+1, len(walls)):
            A1, B1, C1, = walls[i]['params']
            A2, B2, C2, = walls[j]['params']

            det = A1 * B2 - A2 * B1
            if abs(det) < 1e-3: continue

            px = (B1 * C2 - B2 * C1) / det
            py = (A2 * C1 - A1 * C2) / det
            corner = np.array([px, py])

            dist_to_wall1 = np.min(np.linalg.norm(walls[i]['pts'] - corner, axis=1))
            dist_to_wall2 = np.min(np.linalg.norm(walls[j]['pts'] - corner, axis=1))

            if dist_to_wall1 < dist_threshold and dist_to_wall2 < dist_threshold:
                corners.append(corner)
    
    final_corners = []
    for c in corners:
        is_duplicate = False
        for existing in final_corners:
            if np.linalg.norm(c - existing) < corner_threshold:
                is_duplicate = True
                break
        if not is_duplicate:
            final_corners.append(c)
    
    if len(final_corners) == 0:
        return None

    #print(f"[RANSAC] Found {len(walls)} walls and {len(final_corners)} valid corners")
    return np.array(final_corners)

def find_corners_hough(df, max_walls=6, min_points=80, corner_threshold=0.5):
    points = df[['x', 'y']].values
    if len(points) < min_points:
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

# Visualisers
def create_pc_figure(df, c, min_x, min_y, max_x, max_y, draw_max=NBR_POINTS):
    if not df.empty:
        # calculate x and y closest to the lidar
        distances = np.hypot(df["x"].values, df["y"].values)
        i = distances.argmin()

        d = distances[i]
        x = df["x"].values[i]
        y = df["y"].values[i]

        x = np.array([x])
        y = np.array([y])
    else:
        d = 0
        x = np.array([0.0])
        y = np.array([0.0])

    # check number of points
    if len(df) > draw_max:
        df = df.sample(n=draw_max)
    
    fig = px.scatter_3d(
        df, x='x', y='y', z='z',
        color='intensity',
        color_continuous_scale='Plasma',
        range_x=[min_x - 1, max_x + 1],
        range_y=[min_y - 1, max_y + 1],
        opacity=0.4,
        title=f"LiDAR Point Cloud ({len(df):,} pts)",
        template=COLOR["plotly_template"]
    )

    zero = np.array([0.0])
    fig.add_trace(go.Scatter3d(
        x=zero, y=zero, z=zero,
        mode='markers+text',
        marker=dict(size=4, color=COLOR["marker_lidar"], symbol='circle'),
        name='Lidar',
        text=["LiDAR"]
    ))

    fig.add_trace(go.Scatter3d(
        x=x, y=y, z=zero,
        mode='markers+text',
        marker=dict(size=6, color=COLOR["marker_dist"], symbol='cross'),
        name='closest-point',
        text=[f"Dist={round(d,4)}"]
    ))

    z_display = df['z'].mean()

    # Add Corners
    if c is not None:
        fig.add_trace(go.Scatter3d(
            x=c[:, 0], y=c[:, 1], z=[z_display] * len(c),
            mode='markers+text',
            marker=dict(size=6, color=COLOR["marker_corners"], symbol='diamond'),
            name='Corners',
            text=["Corner"] * len(c)
        ))

    fig.update_traces(marker=dict(size=1.5), selector=dict(type='scatter3d', mode='markers'))
    fig.update_layout(
        scene_aspectmode='data', 
        margin=dict(l=0, r=0, b=0, t=40),
        legend=dict(yanchor="top", y=0.99, xanchor="left", x=0.01),
        paper_bgcolor=COLOR["bg_card"],
        plot_bgcolor=COLOR["bg_card"],
        font=dict(color=COLOR["text_main"])
    )
    return fig

def create_imu_figure(df):
    # Time axis
    if 'time_sec' in df.columns:
        t = df["time_sec"] + (df["time_nsec"] / 1e9)
        x_label = "Time (seconds)"
    else:
        t = df["seq"]
        x_label = "Sequence Number"

    # Create subplots
    fig = make_subplots(rows=2, cols=1, 
                        shared_xaxes=True, 
                        vertical_spacing=0.1,
                        subplot_titles=("Linear Acceleration (m/s²)", "Euler Angles (degrees)"))

    # Accelerometer traces
    for col in ["acc_x", "acc_y", "acc_z_corrected"]:
        fig.add_trace(go.Scatter(x=t, y=df[col], name=col, mode='lines'), row=1, col=1)

    # Quaternion traces
    for col in ["roll", "pitch", "yaw"]:
        fig.add_trace(go.Scatter(x=t, y=df[col], name=col, mode='lines'), row=2, col=1)

    fig.update_layout(
        height=700,
        title_text="IMU Sensor Data",
        showlegend=True,
        template=COLOR["plotly_template"],
        paper_bgcolor=COLOR["bg_card"],
        plot_bgcolor=COLOR["bg_card"],
        font=dict(color=COLOR["text_main"])
    )
    fig.update_xaxes(title_text=x_label, row=2, col=1, gridcolor=COLOR["grid_color"])
    fig.update_yaxes(gridcolor=COLOR["grid_color"])
    return fig

def update_text_imu(df, time, z):
    second_df = df[
        (df['time_sec'] >= time) &
        (df['time_sec'] < time + DELTA)
    ]

    if second_df.empty:
        return "No IMU data"

    # Sum accelerations
    acc_x_sum = second_df['acc_x'].mean()
    acc_y_sum = second_df['acc_y'].mean()
    acc_z_sum = second_df['acc_z'].mean() - 9.81 # gravity

    # Sum rotations/quaternions
    qw_sum = second_df['qw'].mean()
    qx_sum = second_df['qx'].mean()
    qy_sum = second_df['qy'].mean()
    qz_sum = second_df['qz'].mean()

    #r = R.from_quat([qx_sum, qy_sum, qz_sum, qw_sum])
    #r_x, r_y, r_z = r.as_euler('xyz', degrees=True)

    return [
        f"LiDAR height: {TRAIN_HEIGHT-z:.2f} m", dash.html.Br(),
        dash.html.Br(),
        f"Acc x: {acc_x_sum:.3f} m/s", dash.html.Br(),
        f"Acc y: {acc_y_sum:.3f} m/s", dash.html.Br(),
        f"Acc z: {acc_z_sum:.3f} m/s", dash.html.Br(),
        dash.html.Br(),
        "Rot x: ... degrees", dash.html.Br(),
        "Rot y: ... degrees", dash.html.Br(),
        "Rot z: ... degrees"
        """
        f"Rot x: {r_x:.3f} degrees", dash.html.Br(),
        f"Rot y: {r_y:.3f} degrees", dash.html.Br(),
        f"Rot z: {r_z:.3f} degrees"
        """
    ]

# Main
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Visualize LiDAR and IMU CSV data via Dash")
    parser.add_argument("--max-pts", type=int, default=NBR_POINTS, help="Max point rows to load")
    parser.add_argument("--port", type=int, default=8050, help="Visualisation backend for point cloud (default: open3d)")
    args = parser.parse_args()

    df_pts = load_csv(POINTS_CSV)
    df_imu = load_csv(IMU_CSV)

    df_imu = create_roll_pitch_yaw(df_imu)
    #print("[Debug] IMU processed")

    # Process data points
    df_pts_process, z = ceiling_process(df_pts)
    df_pts_process = df_pts_process[
            (df_pts_process["x"] < 30) &
            (df_pts_process["y"] < 30) &
            (df_pts_process["z"] < 30)
        ].copy()
    #print("[Debug] Points processed")

    # Pre-calculate time range
    second_min_time = df_pts_process['time'].drop_duplicates().nsmallest(2).iloc[-1]
    second_max_time = df_pts_process['time'].drop_duplicates().nlargest(2).iloc[-1]
    min_x = df_pts_process['x'].min()
    min_y = df_pts_process['y'].min()
    max_x = df_pts_process['x'].max()
    max_y = df_pts_process['y'].max()
    #print("[Debug] Min/Max calculated")

    fig_imu = create_imu_figure(df_imu)
    #print("[Debug] Figure IMU created")

    # Initialize Dash App
    app = dash.Dash(__name__)

    app.layout = dash.html.Div(
        style={
            'backgroundColor': COLOR["bg_app"],
            'color': COLOR["text_main"],
            'padding': '30px',
            'minHeight': '100vh',
            'fontFamily': 'Inter, sans-serif',
        }, children=[
            dash.html.H1("LiDAR & IMU", style={'textAlign': 'center', 'color': COLOR["text_main"]}),
            
            # --- Control Panel ---
            dash.html.Div([
                dash.html.Div(
                    "Show all data points",
                    style={'color': COLOR["text_main"], 'fontWeight': '500'}
                ),
                daq.ToggleSwitch(
                    id='switch',
                    value=False,
                    color=COLOR["switch"],
                ),
                dash.html.Div(
                    [dash.html.Br(), "Timeline"],
                    style={'color': COLOR["text_main"], 'fontWeight': '500'}
                ),
                dash.dcc.Slider(
                    id='time-slider',
                    className="custom-slider",
                    min=second_min_time,
                    max=second_max_time,
                    value=second_min_time,
                    marks=None,
                    step=DELTA
                ),
                dash.html.P(
                    id='imu-info',
                    children=[
                        f"Height: {TRAIN_HEIGHT-z:.2f} m", dash.html.Br(),
                        dash.html.Br(),
                        "Acc x: .. m/s", dash.html.Br(),
                        "Acc y: .. m/s", dash.html.Br(),
                        "Acc z: .. m/s", dash.html.Br(),
                        dash.html.Br(),
                        "Rot x: .. degrees", dash.html.Br(),
                        "Rot y: .. degrees", dash.html.Br(),
                        "Rot z: .. degrees"
                    ],
                    style={
                        'lineHeight': '1.8',
                        'fontSize': '14px',
                        'color': COLOR["text_sub"]
                    }
                )
            ],style={
                'padding': '20px',
                'backgroundColor': COLOR["bg_card"],
                'borderRadius': '10px',
                'marginBottom': '20px',
                'border': COLOR["border"]
            }
        ),

        # --- Visuals ---
        dash.html.Div(
            dash.dcc.Graph(id='fig_pts', style={'height': '70vh'}), 
            style={
                'padding': '10px',
                'backgroundColor': COLOR["bg_card"],
                'border': COLOR["border"],
                'marginBottom': '20px',
                'borderRadius': '10px'
            }
        ),

        dash.html.Div(
            dash.dcc.Graph(figure=fig_imu),
            style={
                'padding': '10px',
                'backgroundColor': COLOR["bg_card"],
                'border': COLOR["border"],
                'borderRadius': '10px'
            }
        )
    ])
    #print("[Debug] App init")

    @dash.callback(
        dash.Output('imu-info', 'children'),
        dash.Output('fig_pts', 'figure'),
        dash.Input('time-slider', 'value'),
        dash.Input('switch', 'value')
    )
    def update_figures(selected_time, no_time):
        if no_time:
            df_pts_process, z = ceiling_process(df_pts)
            c = corners_process(df_pts_process, z)
            fig_pts = create_pc_figure(df_pts, c, min_x, min_y, max_x, max_y, args.max_pts)
            return "", fig_pts
        else:
            if (selected_time == second_min_time):
                filtered_df_pts = df_pts[df_pts['time'].between(selected_time, selected_time + DELTA)].copy()
            else:
                filtered_df_pts = df_pts[df_pts['time'].between(selected_time - DELTA, selected_time + DELTA)].copy()
            #print("[Debug] df filtred")
            df_pts_deskew = deskew_points(filtered_df_pts, df_imu)
            #print("[Debug] df deskew")
            df_pts_process, z = ceiling_process(df_pts_deskew)
            #print("[Debug] ceiling found")
            c = corners_process(df_pts_process, z)
            #print("[Debug] corners found")
            fig_pts = create_pc_figure(df_pts_process, c, min_x, min_y, max_x, max_y, args.max_pts)
            #print("[Debug] Figure pts created")
            return update_text_imu(df_imu, selected_time, z), fig_pts

    # Run server on 0.0.0.0 to make it accessible on the local network
    print(f"\n--- Server starting on port {args.port} ---")
    app.run(debug=False, host='0.0.0.0', port=args.port)