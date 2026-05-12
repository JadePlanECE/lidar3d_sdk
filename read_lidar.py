import argparse
import sys
import os
import numpy as np
import pandas as pd
import plotly.express as px
import plotly.graph_objects as go
from plotly.subplots import make_subplots
import scipy
from scipy.spatial import ConvexHull
import dash

# Constants
POINTS_CSV = "./data/points.csv"
IMU_CSV = "./data/imu.csv"
NBR_POINTS = 200000

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
    corners_km = find_corners_kmean(df_walls)
    
    return df_walls, corners_pca, corners_grad, corners_km

def find_ceiling(df):
    """
    Finds the ceiling height by looking for the statistical mode of the Z-axis
    We must put more weight to the points closer to x=0 and y=0
    """
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
    print(f"[Process] Wall strip [{lower:.2f}, {upper:.2f}]: {len(strip):,} points")
    return strip

def find_corners_eigenvectors(df):
    """
    Finds corners using Principal Component Analysis (PCA) logic.
    Assumes a rectangular room.
    """
    # 1. Position at center
    coords = df[['x', 'y']].values
    center = coords.mean(axis=0)
    centered = coords - center

    cov = np.cov(centered.T)
    _, eigenvectors = np.linalg.eigh(cov)
    
    aligned = centered @ eigenvectors
    
    lo_x, hi_x = np.percentile(aligned[:,0], [1,99])
    lo_y, hi_y = np.percentile(aligned[:,1], [1,99])
    
    aligned_corners = np.array([
        [lo_x, lo_y],
        [hi_x, lo_y],
        [hi_x, hi_y],
        [lo_x, hi_y]
    ])
    
    world_corners = (aligned_corners @ eigenvectors.T) + center
    
    print(f"[PCA] Found 4 corners using principal axes")
    return world_corners

def find_corners_derivate(df, bins=360):
    """
    Finds corners by detecting sharp changes in the radial distance gradient in polar coordinates.
    """
    coords = df[['x', 'y']].values
    center = coords.mean(axis=0)
    x_c = df['x'].values - center[0]
    y_c = df['y'].values - center[1]

    r = np.sqrt(x_c**2 + y_c**2)
    theta = np.arctan2(y_c, x_c)

    df_polar = pd.DataFrame({'theta': theta, 'r': r})
    bin_edges = np.linspace(-np.pi, np.pi, bins + 1)
    bin_centers = 0.5 * (bin_edges[:-1] + bin_edges[1:])
    df_polar['theta_bin'] = pd.cut(df_polar['theta'], bins=bin_edges, labels=False)

    def near_median(g):
        q = g.quantile(0.10)
        return g[g <= q].median() if len(g) else np.nan

    profile_series = df_polar.groupby('theta_bin', observed=False)['r'].apply(near_median)
    profile = profile_series.reindex(range(bins)).interpolate(method='linear').fillna(method='bfill').fillna(method='ffill').values

    from scipy.ndimage import gaussian_filter1d
    profile_smooth = gaussian_filter1d(profile, sigma=3)
    diff = np.abs(np.gradient(profile_smooth))

    min_sep = int(bins * (70 / 360))

    peaks = []
    diff_copy = diff.copy()
    for _ in range(4):
        idx = int(np.argmax(diff_copy))
        peaks.append(idx)
        # Suppress a window around this peak
        lo = max(0, idx - min_sep)
        hi = min(bins, idx + min_sep)
        diff_copy[lo:hi] = 0
    
    detected_corners = []
    for idx in peaks:
        angle = bin_centers[idx]
        dist = profile_smooth[idx]
        detected_corners.append([
            dist * np.cos(angle) + center[0],
            dist * np.sin(angle) + center[1],
        ])

    print(f"[Derivative] Detected {len(detected_corners)} corners via radial gradient")
    return np.array(detected_corners)

def find_corners_kmean(df, max_itera=300, tol=1e-4):
    X = df[['x', 'y']].values
    
    hull = ConvexHull(X)
    X_hull = X[hull.vertices]
    
    mean = X_hull.mean(axis=0)
    std  = X_hull.std(axis=0) + 1e-8
    Xn   = (X_hull - mean) / std

    np.random.seed(0)
    K = 4
    centroids = [Xn[np.random.randint(len(Xn))]]
    for _ in range(K - 1):
        dists = np.min(
            np.stack([np.sum((Xn - c) ** 2, axis=1) for c in centroids], axis=1),
            axis=1,
        )
        probs = dists / dists.sum()
        centroids.append(Xn[np.random.choice(len(Xn), p=probs)])
    centroids = np.array(centroids)

    for itera in range(1, max_itera + 1):
        diffs = Xn[:, None, :] - centroids[None, :, :]   # (N, K, 2)
        labels = np.argmin((diffs ** 2).sum(axis=2), axis=1)

        new_centroids = np.array([
            Xn[labels == k].mean(axis=0) if (labels == k).any() else centroids[k]
            for k in range(K)
        ])

        if np.allclose(centroids, new_centroids, atol=tol):
            centroids = new_centroids
            break
        centroids = new_centroids

    world_centroids = centroids * std + mean

    print(f"[Kmean] Found 4 corners using Kmean algo in {itera} iterations")
    return world_centroids

# Visualisers
def create_pc_figure(df, c_pca=None, c_grad=None, c_km=None, draw_max=NBR_POINTS):
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
            marker=dict(size=6, color='red', symbol='diamond'),
            name='Corners (PCA)',
            text=["PCA Corner"] * len(c_pca)
        ))

    # Add Gradient Corners (Blue)
    if c_grad is not None:
        fig.add_trace(go.Scatter3d(
            x=c_grad[:, 0], y=c_grad[:, 1], z=[z_display] * len(c_grad),
            mode='markers+text',
            marker=dict(size=6, color='blue', symbol='circle'),
            name='Corners (Gradient)',
            text=["Grad Corner"] * len(c_grad)
        ))

    # Add Me Corners (Green)
    if c_km is not None:
        fig.add_trace(go.Scatter3d(
            x=c_km[:, 0], y=c_km[:, 1], z=[z_display] * len(c_km),
            mode='markers+text',
            marker=dict(size=6, color='green', symbol='cross'),
            name='Corners (Kmean)',
            text=["Kmean Corner"] * len(c_km)
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
    df_pts, c_pca, c_grad, c_km = process_data_points(df_pts)

    # Generate Figures
    fig_pc = create_pc_figure(df_pts, c_pca, c_grad, c_km)
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
