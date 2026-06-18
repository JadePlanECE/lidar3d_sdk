import numpy as np
import dash
import dash_daq as daq
import plotly.express as px
import plotly.graph_objects as go
from plotly.subplots import make_subplots
from scipy.spatial.transform import Rotation as R
import process

class Visualisation:
    def __init__(self, df_pts, df_imu, port, max_pts, darkmode, delta):
        self.df_pts = df_pts
        self.df_imu = df_imu
        self.port = port
        self.max_pts = max_pts
        self.delta = delta

        self.processeur = process.Process()
        min_time, max_time, min_x, max_x, min_y, max_y = self.processeur.calculate_time(self.df_pts)
        self.min_time = min_time
        self.max_time = max_time
        if min_time == max_time:
            print("[Warning] min time slider = max time slider")
            print(f"min time slider: {min_time}")
            print(f"max time slider: {max_time}")
            print(f"delta: {delta}")
        elif min_time == max_time + delta:
            print("[Warning] min time slider = max time slider + delta")
            print(f"min time slider: {min_time}")
            print(f"max time slider: {max_time}")
            print(f"delta: {delta}")
        elif min_time + delta > max_time:
            print("[Warning] min time slider + delta > max time slider")
            print(f"min time slider: {min_time}")
            print(f"max time slider: {max_time}")
            print(f"delta: {delta}")
        self.min_x = min_x
        self.max_x = max_x
        self.min_y = min_y
        self.max_y = max_y

        self.train_height = 3.0
        themes = {
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
        self.colors = themes["dark"] if darkmode else themes["light"]

        self.app = dash.Dash(__name__)

    def init_app(self, fig_imu):
        self.app.layout = dash.html.Div(
            style={
                'backgroundColor': self.colors["bg_app"],
                'color': self.colors["text_main"],
                'padding': '30px',
                'minHeight': '100vh',
                'fontFamily': 'Inter, sans-serif',
            }, children=[
                dash.html.H1("LiDAR & IMU", style={'textAlign': 'center', 'color': self.colors["text_main"]}),
                
                # --- Control Panel ---
                dash.html.Div([
                    dash.html.Div(
                        "Show all data points",
                        style={'color': self.colors["text_main"], 'fontWeight': '500'}
                    ),
                    daq.ToggleSwitch(
                        id='switch',
                        value=False,
                        color=self.colors["switch"],
                    ),
                    dash.html.Div(
                        [dash.html.Br(), "Timeline"],
                        style={'color': self.colors["text_main"], 'fontWeight': '500'}
                    ),
                    dash.dcc.Slider(
                        id='time-slider',
                        className="custom-slider",
                        min=self.min_time,
                        max=self.max_time,
                        value=self.min_time,
                        marks=None,
                        step=self.delta
                    ),
                    dash.html.P(
                        id='imu-info',
                        children=[
                            f"Height: ... m", dash.html.Br(),
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
                            'color': self.colors["text_sub"]
                        }
                    )
                ],style={
                    'padding': '20px',
                    'backgroundColor': self.colors["bg_card"],
                    'borderRadius': '10px',
                    'marginBottom': '20px',
                    'border': self.colors["border"]
                }
            ),

            # --- Visuals ---
            dash.html.Div(
                dash.dcc.Graph(id='fig_pts', style={'height': '70vh'}), 
                style={
                    'padding': '10px',
                    'backgroundColor': self.colors["bg_card"],
                    'border': self.colors["border"],
                    'marginBottom': '20px',
                    'borderRadius': '10px'
                }
            ),

            dash.html.Div(
                dash.dcc.Graph(figure=fig_imu),
                style={
                    'padding': '10px',
                    'backgroundColor': self.colors["bg_card"],
                    'border': self.colors["border"],
                    'borderRadius': '10px'
                }
            )
        ])

    def create_pts_figure(self, df, c):
        if not df.empty:
            # calculate x and y closest and farthest to the lidar
            distances = np.hypot(df["x"].values, df["y"].values)
            i_min = distances.argmin()
            i_max = distances.argmax()

            d_min = distances[i_min]
            d_max = distances[i_max]

            x_min = df["x"].values[i_min]
            y_min = df["y"].values[i_min]
            x_max = df["x"].values[i_max]
            y_max = df["y"].values[i_max]

            x_min = np.array([x_min])
            y_min = np.array([y_min])
            x_max = np.array([x_max])
            y_max = np.array([y_max])
        else:
            d_min = 0
            d_max = 0
            x_min = np.array([0.0])
            y_min = np.array([0.0])
            x_max = np.array([0.0])
            y_max = np.array([0.0])

        # check number of points
        if len(df) > self.max_pts:
            df = df.sample(n=self.max_pts)
        
        fig = px.scatter_3d(
            df, x='x', y='y', z='z',
            color='intensity',
            color_continuous_scale='Plasma',
            range_x=[self.min_x - 1, self.max_x + 1],
            range_y=[self.min_y - 1, self.max_y + 1],
            opacity=0.4,
            title=f"LiDAR Point Cloud ({len(df):,} pts)",
            template=self.colors["plotly_template"]
        )

        zero = np.array([0.0])
        fig.add_trace(go.Scatter3d(
            x=zero, y=zero, z=zero,
            mode='markers+text',
            marker=dict(size=4, color=self.colors["marker_lidar"], symbol='circle'),
            name='Lidar',
            text=["LiDAR"]
        ))

        fig.add_trace(go.Scatter3d(
            x=x_min, y=y_min, z=zero,
            mode='markers+text',
            marker=dict(size=6, color=self.colors["marker_dist"], symbol='cross'),
            name='closest-point',
            text=[f"Dist={round(d_min,4)}"]
        ))
        fig.add_trace(go.Scatter3d(
            x=x_max, y=y_max, z=zero,
            mode='markers+text',
            marker=dict(size=6, color=self.colors["marker_dist"], symbol='cross'),
            name='farthest-point',
            text=[f"Dist={round(d_max,4)}"]
        ))

        z_display = df['z'].mean()

        # Add Corners
        if c is not None:
            fig.add_trace(go.Scatter3d(
                x=c[:, 0], y=c[:, 1], z=[z_display] * len(c),
                mode='markers+text',
                marker=dict(size=6, color=self.colors["marker_corners"], symbol='diamond'),
                name='Corners',
                text=["Corner"] * len(c)
            ))

        fig.update_traces(marker=dict(size=1.5), selector=dict(type='scatter3d', mode='markers'))
        fig.update_layout(
            scene_aspectmode='data', 
            margin=dict(l=0, r=0, b=0, t=40),
            legend=dict(yanchor="top", y=0.99, xanchor="left", x=0.01),
            paper_bgcolor=self.colors["bg_card"],
            plot_bgcolor=self.colors["bg_card"],
            font=dict(color=self.colors["text_main"])
        )
        return fig

    def create_imu_figure(self, df):
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
            template=self.colors["plotly_template"],
            paper_bgcolor=self.colors["bg_card"],
            plot_bgcolor=self.colors["bg_card"],
            font=dict(color=self.colors["text_main"])
        )
        fig.update_xaxes(title_text=x_label, row=2, col=1, gridcolor=self.colors["grid_color"])
        fig.update_yaxes(gridcolor=self.colors["grid_color"])
        return fig

    def update_text_imu(self, df, time, z):
        second_df = df[
            (df['time_sec'] >= time) &
            (df['time_sec'] < time + self.delta)
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

        r = R.from_quat([qx_sum, qy_sum, qz_sum, qw_sum])
        r_x, r_y, r_z = r.as_euler('xyz', degrees=True)

        return [
            f"LiDAR height: {self.train_height-z:.2f} m", dash.html.Br(),
            dash.html.Br(),
            f"Acc x: {acc_x_sum:.3f} m/s", dash.html.Br(),
            f"Acc y: {acc_y_sum:.3f} m/s", dash.html.Br(),
            f"Acc z: {acc_z_sum:.3f} m/s", dash.html.Br(),
            dash.html.Br(),
            f"Rot x: {r_x:.3f} degrees", dash.html.Br(),
            f"Rot y: {r_y:.3f} degrees", dash.html.Br(),
            f"Rot z: {r_z:.3f} degrees"
        ]
    
    def visualisation_data(self):
        fig_imu = self.create_imu_figure(self.df_imu)
        self.init_app(fig_imu)

        @dash.callback(
            dash.Output('imu-info', 'children'),
            dash.Output('fig_pts', 'figure'),
            dash.Input('time-slider', 'value'),
            dash.Input('switch', 'value')
        )
        def update_figures(selected_time, no_time):
            df_pts_deskew = self.processeur.deskew_points(self.df_pts, self.df_imu)
            if no_time:
                df_pts_process, z = self.processeur.ceiling_process(df_pts_deskew)
                c = self.processeur.corners_process(df_pts_process, z)
                fig_pts = self.create_pts_figure(df_pts_process, c)
                return "", fig_pts
            else:
                if (selected_time == self.min_time):
                    filtered_df_pts = df_pts_deskew[df_pts_deskew['time'].between(selected_time, selected_time + self.delta)].copy()
                    if (filtered_df_pts.equals(df_pts_deskew)):
                        print("df_pts is not filtred")
                else:
                    filtered_df_pts = df_pts_deskew[df_pts_deskew['time'].between(selected_time - self.delta, selected_time + self.delta)].copy()
                    if (filtered_df_pts.equals(df_pts_deskew)):
                        print("df_pts is not filtred")
                df_pts_process, z = self.processeur.ceiling_process(df_pts_deskew)
                c = self.processeur.corners_process(df_pts_process, z)
                fig_pts = self.create_pts_figure(df_pts_process, c)
                return self.update_text_imu(self.df_imu, selected_time, z), fig_pts

        print(f"\n--- Server starting on port {self.port} ---")
        self.app.run(debug=False, host='0.0.0.0', port=self.port)
