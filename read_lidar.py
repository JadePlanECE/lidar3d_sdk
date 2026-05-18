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
import dash
import dash_daq as daq

# Constants
#POINTS_CSV = "./data/points_time.csv"
#IMU_CSV = "./data/imu_time.csv"
POINTS_CSV = "./data/points.csv"
IMU_CSV = "./data/imu.csv"

NBR_POINTS = 100000
TRAIN_HEIGHT = 3.0

# Load data
def load_points(path=POINTS_CSV):
    if not os.path.exists(path):
        sys.exit(f"[Error] File not found {path}\n")
    
    df = pd.read_csv(path)
    print(f"[Points] Loaded {len(df):,} rows from {path}")
    return df

def load_imu(path=IMU_CSV):
    if not os.path.exists(path):
        sys.exit(f"[Error] File not found {path}\n")
    
    df = pd.read_csv(path)
    print(f"[IMU] Loaded {len(df):,} rows from {path}")
    return df

# Process
def process_data_points(df):
    df = df.iloc[:-1] #drop the last line (always unfinished)
    ceiling_z = find_ceiling(df)
    df_no_ceiling = erase_ceiling(df, ceiling_z)
    return df_no_ceiling, ceiling_z

def corners(df, z):
    df_walls = find_walls(df, z)

    if df_walls.empty:
        print("[Error] No wall points found. Adjust your Z-thresholds.")
        return None

    return find_corners(df_walls)

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
    
    print(f"[Process] Detected ceiling at Z = {ceiling_z:.2f}")
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

    #print(f"[RANSAC] Found {len(walls)} walls and {len(final_corners)} valid corners")
    return np.array(final_corners)

# Visualisers
def create_pc_figure(df, c, min_x, min_y, max_x, max_y, draw_max=NBR_POINTS):
    if len(df) > draw_max:
        df = df.sample(n=draw_max)
    
    fig = px.scatter_3d(
        df, x='x', y='y', z='z',
        color='intensity',
        color_continuous_scale='Plasma',
        range_x=[min_x - 1, max_x + 1],
        range_y=[min_y - 1, max_y + 1],
        opacity=0.4,
        title=f"LiDAR Point Cloud ({len(df):,} pts)"
    )

    zero = np.array([0.0])
    fig.add_trace(go.Scatter3d(
        x=zero, y=zero, z=zero,
        mode='markers+text',
        marker=dict(size=4, color='black', symbol='circle'),
        name='Lidar',
        text=["LiDAR"]
    ))

    z_display = df['z'].mean()

    # Add Ransac Corners (Green)
    if c is not None:
        fig.add_trace(go.Scatter3d(
            x=c[:, 0], y=c[:, 1], z=[z_display] * len(c),
            mode='markers+text',
            marker=dict(size=6, color='green', symbol='diamond'),
            name='Corners (Ransac)',
            text=["Ransac Corner"] * len(c)
        ))

    fig.update_traces(marker=dict(size=1.5), selector=dict(type='scatter3d', mode='markers'))
    fig.update_layout(
        scene_aspectmode='data', 
        margin=dict(l=0, r=0, b=0, t=40),
        legend=dict(yanchor="top", y=0.99, xanchor="left", x=0.01)
    )
    return fig

def create_imu_figure(df):
    # Create subplots: one for Accel, one for Quaternions
    fig = make_subplots(rows=2, cols=1, 
                        shared_xaxes=True, 
                        vertical_spacing=0.1,
                        subplot_titles=("Linear Acceleration (m/s²)", "Quaternions"))

    if 'time_sec' in df.columns:
        t = df["time_sec"] + (df["time_nsec"] / 1e9)
        x_label = "Time (seconds)"
    else:
        t = df["seq"]
        x_label = "Sequence Number"

    # Accelerometer traces
    for col in ["acc_x", "acc_y", "acc_z"]:
        fig.add_trace(go.Scatter(x=t, y=df[col], name=col, mode='lines'), row=1, col=1)

    # Quaternion traces
    for col in ["qw", "qx", "qy", "qz"]:
        fig.add_trace(go.Scatter(x=t, y=df[col], name=col, mode='lines'), row=2, col=1)

    fig.update_layout(height=600, title_text="IMU Sensor Data", showlegend=True)
    fig.update_xaxes(title_text=x_label, row=2, col=1)
    return fig

def update_text_imu(df, time):
    second_df = df[
        (df['time_sec'] >= time) &
        (df['time_sec'] < time + 1)
    ]

    if second_df.empty:
        return "No IMU data"

    # Sum accelerations
    acc_x_sum = second_df['acc_x'].sum()
    acc_y_sum = second_df['acc_y'].sum()
    acc_z_sum = second_df['acc_z'].sum()

    # Sum rotations/quaternions
    qw_sum = second_df['qw'].sum()
    qx_sum = second_df['qx'].sum()
    qy_sum = second_df['qy'].sum()
    qz_sum = second_df['qz'].sum()

    return [
        f"Acc x: {acc_x_sum:.3f}", dash.html.Br(),
        f"Acc y: {acc_y_sum:.3f}", dash.html.Br(),
        f"Acc z: {acc_z_sum:.3f}", dash.html.Br(),
        dash.html.Br(),
        f"Rot w: {qw_sum:.3f}", dash.html.Br(),
        f"Rot x: {qx_sum:.3f}", dash.html.Br(),
        f"Rot y: {qy_sum:.3f}", dash.html.Br(),
        f"Rot z: {qz_sum:.3f}"
    ]

# Main
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Visualise LiDAR and IMU CSV data via Dash")
    parser.add_argument("--max-pts", type=int, default=NBR_POINTS, help="Max point rows to load")
    parser.add_argument("--port", type=int, default=8050, help="Visualisation backend for point cloud (default: open3d)")
    args = parser.parse_args()

    df_pts = load_points()
    df_imu = load_imu()

    # Process data points
    #df_pts['time'] = pd.to_datetime(df_pts['time'], unit='s')
    df_pts, z = process_data_points(df_pts)

    # Pre-calculate time range
    min_time = df_pts['time'].min()
    max_time = df_pts['time'].max()
    second_max_time = df_pts['time'].drop_duplicates().nlargest(2).iloc[-1]
    min_x = df_pts['x'].min()
    min_y = df_pts['y'].min()
    max_x = df_pts['x'].max()
    max_y = df_pts['y'].max()

    # Initialize Dash App
    app = dash.Dash(__name__)

    fig_imu = create_imu_figure(df_imu)

    app.layout = dash.html.Div(style={'backgroundColor': 'white', 'color': 'black', 'padding': '20px'}, children=[
        dash.html.H1("LiDAR & IMU", style={'textAlign': 'center'}),
        
        # --- Control Panel ---
        dash.html.Div([
            dash.dcc.Slider(
                id='time-slider',
                min=min_time,
                max=second_max_time,
                value=min_time,
                marks=None,
                step=None
            ),
            daq.ToggleSwitch(
                id='switch',
                label="All data points",
                value=False
            ),
            dash.html.P(
                id='lidar-height',
                children=f"Lidar height: {TRAIN_HEIGHT-z}"
            ),
            dash.html.P(
                id='imu-info',
                children=[
                    "Acc x: ?", dash.html.Br(),
                    "Acc y: ?", dash.html.Br(),
                    "Acc z: ?", dash.html.Br(),
                    "Rot w: ?", dash.html.Br(),
                    "Rot x: ?", dash.html.Br(),
                    "Rot y: ?", dash.html.Br(),
                    "Rot z: ?"
                ]
            )
            ],style={'padding': '20px', 'backgroundColor': 'white', 'borderRadius': '10px', 'marginBottom': '20px'}
        ),

        # --- Visuals ---
        dash.html.Div(
            dash.dcc.Graph(id='fig_pts', style={'height': '70vh'}), 
            style={'padding': '10px', 'border': '1px solid #444', 'marginBottom': '20px'}
        ),

        dash.html.Div(
            dash.dcc.Graph(figure=fig_imu),
            style={'padding': '10px', 'border': '1px solid #444'}
        )
    ])

    @dash.callback(
        dash.Output('imu-info', 'children'),
        dash.Output('fig_pts', 'figure'),
        dash.Input('time-slider', 'value'),
        dash.Input('switch', 'value')
    )
    def update_figures(selected_time, no_time):
        if no_time:
            c = corners(df_pts, z)
            fig_pts = create_pc_figure(df_pts, c, min_x, min_y, max_x, max_y, args.max_pts)
            return "", fig_pts
        else:
            filtered_df_pts = df_pts[df_pts['time'].between(selected_time - 1, selected_time + 1)].copy()
            c = corners(df_pts, z)
            fig_pts = create_pc_figure(filtered_df_pts, c, min_x, min_y, max_x, max_y, args.max_pts)
            return update_text_imu(df_imu, selected_time), fig_pts

    # Run server on 0.0.0.0 to make it accessible on the local network
    print(f"\n--- Server starting on port {args.port} ---")
    app.run(debug=False, host='0.0.0.0', port=args.port)
