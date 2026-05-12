import argparse
import sys
import os
import numpy as np
import pandas as pd
import plotly.express as px
import plotly.graph_objects as go
from plotly.subplots import make_subplots
import matplotlib.pyplot as plt
import dash

# Constants
POINTS_CSV = "./data/points.csv"
IMU_CSV = "./data/imu.csv"
NBR_POINTS = 1000

# Load data
def load_points(path=POINTS_CSV, max_pts=None, ring=None):
    if not os.path.exists(path):
        sys.exit(f"[Error] File not found {path}\n")
    
    df = pd.read_csv(path, nrows=max_pts)
    print(f"[Points] Loaded {len(df):,} rows from {path}")

    if ring is not None:
        df = df[df["ring"] == ring]
        print(f" - Filtred to ring={ring} {len(df):,} points")
    
    return df

def load_imu(path=IMU_CSV):
    if not os.path.exists(path):
        sys.exit(f"[Error] File not found {path}\n")
    
    df = pd.read_csv(path)
    print(f"[IMU] Loaded {len(df):,} rows from {path}")
    return df

# Process
def process_data_points(df):
    ceiling_z = find_ceiling(df)
    df_no_ceiling = erase_ceiling(df, ceiling_z)
    df_walls = find_walls(df_no_ceiling, ceiling_z)
    
    corners_pca = find_corners_eigenvectors(df_walls)
    corners_grad = find_corners_derivate(df_walls)
    
    return df_walls, corners_pca, corners_grad

def find_ceiling(df):
    """
    Finds the ceiling height by looking for the statistical mode of the Z-axis
    We must put more weight to the points closer to x=0 and y=0
    """
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
    return df[df['z'] < (z - threshold)].copy()

def find_walls(df, ceiling, margin=0.1):
    """
    Classifies points as walls if they are in the middle between :
    - the ceiling (erased)
    - 5% of the lowest points
    df is then a strip (in theory, the shape of the room)
    """
    lowest_5 = df['z'].quantile(0.05)
    middle = (ceiling - lowest_5) / 2
    print(f"[Process] Middle: {middle}\n")

    # Calcul des bornes
    lower = middle * (1 - margin)
    upper = middle * (1 + margin)

    strip = df[df['z'].between(lower, upper)]

    return strip

"""
    Finds corners by looking at the intersection of high-density vertical segments
    1. Position yourself at the center of the room (instead of (0;0))
    2. switch to polar coordinates
    3. Calculate the eigenvectors to find the 4 corners (rectangular room)
"""
def find_corners_eigenvectors(df):
    """
    Finds corners using Principal Component Analysis (PCA) logic.
    Assumes a rectangular room.
    """
    # 1. Position at center
    coords = df[['x', 'y']].values
    center = coords.mean(axis=0)
    centered_coords = coords - center

    # 2. Calculate Covariance Matrix and Eigenvectors
    # This tells us the 'rotation' of the room relative to the sensor
    cov = np.cov(centered_coords.T)
    eigenvalues, eigenvectors = np.linalg.eigh(cov)
    
    # 3. Project points onto the eigenvectors (aligning the room to axes)
    aligned_coords = centered_coords @ eigenvectors
    
    # 4. Find the min/max in the aligned space (the bounds of the rectangle)
    min_x, min_y = aligned_coords.min(axis=0)
    max_x, max_y = aligned_coords.max(axis=0)
    
    # 5. Define the 4 corners in aligned space
    aligned_corners = np.array([
        [min_x, min_y],
        [max_x, min_y],
        [max_x, max_y],
        [min_x, max_y]
    ])
    
    # 6. Rotate corners back to original space
    world_corners = (aligned_corners @ eigenvectors.T) + center
    
    print(f"[PCA] Found 4 corners using principal axes")
    return world_corners

"""
    Finds corners by looking at the intersection of high-density vertical segments
    1. Position yourself at the center of the room (instead of (0;0))
    2. switch to polar coordinates
    3. Calculate the derivative of the wall dentsity changes sharply
"""
def find_corners_derivate(df, bins=360):
    """
    Finds corners by detecting sharp changes in the radial distance gradient in polar coordinates.
    """
    # 1. Position at center
    coords = df[['x', 'y']].values
    center = coords.mean(axis=0)
    x_c = df['x'] - center[0]
    y_c = df['y'] - center[1]

    # 2. Switch to polar coordinates
    r = np.sqrt(x_c**2 + y_c**2)
    theta = np.arctan2(y_c, x_c)

    # 3. Bin the data to get a smooth "wall profile"
    # We take the median radius for every degree to ignore noise
    df_polar = pd.DataFrame({'theta': theta, 'r': r})
    df_polar['theta_bin'] = pd.cut(df_polar['theta'], bins=np.linspace(-np.pi, np.pi, bins))
    profile = df_polar.groupby('theta_bin', observed=True)['r'].median().interpolate()

    # 4. Calculate the derivative (gradient) of the radius
    # In a rectangle, dr/dtheta peaks at corners
    diff = np.abs(np.gradient(profile.values))
    
    # 5. Find indices of the 4 largest peaks (should be roughly 90 deg apart)
    # We use a simple sort here, but in production, use scipy.signal.find_peaks
    peak_indices = np.argsort(diff)[-4:]
    
    # 6. Map back to XY
    detected_corners = []
    bin_centers = np.linspace(-np.pi, np.pi, bins)
    for idx in peak_indices:
        angle = bin_centers[idx]
        dist = profile.values[idx]
        detected_corners.append([
            dist * np.cos(angle) + center[0],
            dist * np.sin(angle) + center[1]
        ])

    print(f"[Derivative] Detected {len(detected_corners)} corners via radial gradient")
    return np.array(detected_corners)

# Visualisers
def create_pc_figure(df, c_pca=None, c_grad=None, draw_max=NBR_POINTS):
    if len(df) > draw_max:
        df = df.sample(n=draw_max)
    
    fig = px.scatter_3d(
        df, x='x', y='y', z='z',
        color='z',
        color_continuous_scale='Plasma',
        #range_color=[0, 3],
        opacity=0.5,
        title=f"LiDAR Point Cloud ({len(df):,} pts)"
    )

    z_display = df['z'].mean()

    # Add PCA Corners (Red)
    if c_pca is not None:
        fig.add_trace(go.Scatter3d(
            x=c_pca[:, 0], y=c_pca[:, 1], z=[z_display] * len(c_pca),
            mode='markers+text',
            marker=dict(size=8, color='red', symbol='diamond'),
            name='Corners (PCA)',
            text=["PCA Corner"] * len(c_pca)
        ))

    # Add Gradient Corners (Blue)
    if c_grad is not None:
        fig.add_trace(go.Scatter3d(
            x=c_grad[:, 0], y=c_grad[:, 1], z=[z_display] * len(c_grad),
            mode='markers+text',
            marker=dict(size=8, color='cyan', symbol='circle'),
            name='Corners (Gradient)',
            text=["Grad Corner"] * len(c_grad)
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

    t = df["seq"]

    # Accelerometer traces
    for col in ["acc_x", "acc_y", "acc_z"]:
        fig.add_trace(go.Scatter(x=t, y=df[col], name=col, mode='lines'), row=1, col=1)

    # Quaternion traces
    for col in ["qw", "qx", "qy", "qz"]:
        fig.add_trace(go.Scatter(x=t, y=df[col], name=col, mode='lines'), row=2, col=1)

    fig.update_layout(height=600, title_text="IMU Sensor Data", showlegend=True)
    fig.update_xaxes(title_text="Sequence Number", row=2, col=1)
    return fig

# Main
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Visualise LiDAR and IMU CSV data via Dash")
    parser.add_argument("--ring", type=int, default=None, help="Show only this ring index")
    parser.add_argument("--max-pts", type=int, default=None, help="Max point rows to load")
    parser.add_argument("--port", type=int, default=8050, help="Visualisation backend for point cloud (default: open3d)")
    args = parser.parse_args()

    df_pts = load_points(max_pts=args.max_pts, ring=args.ring)
    df_imu = load_imu()

    # Process data points
    df_pts, c_pca, c_grad = process_data_points(df_pts)

    # Generate Figures
    fig_pc = create_pc_figure(df_pts, c_pca, c_grad)
    fig_imu = create_imu_figure(df_imu)

    # Initialize Dash App
    app = dash.Dash(__name__)

    app.layout = dash.html.Div(style={'backgroundColor': '#1e1e1e', 'color': 'white', 'padding': '20px'}, children=[
        dash.html.H1("LiDAR & IMU", style={'textAlign': 'center'}),
        
        dash.html.Div([
            dash.html.H3("3D Point Cloud"),
            dash.dcc.Graph(figure=fig_pc, style={'height': '70vh'})
        ], style={'padding': '10px', 'border': '1px solid #444', 'marginBottom': '20px'}),

        dash.html.Div([
            dash.html.H3("IMU Timeseries"),
            dash.dcc.Graph(figure=fig_imu)
        ], style={'padding': '10px', 'border': '1px solid #444'})
    ])

    # Run server on 0.0.0.0 to make it accessible on the local network
    print(f"\n--- Server starting on port {args.port} ---")
    app.run(debug=False, host='0.0.0.0', port=args.port)
