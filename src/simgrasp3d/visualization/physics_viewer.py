"""比較幾何軟管與 MuJoCo cable 形狀、接觸力及能量。"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import plotly.graph_objects as go
from plotly.subplots import make_subplots

from simgrasp3d.models.motion import TrajectoryData
from simgrasp3d.models.physics import PhysicsSweepData


def build_physics_comparison_figure(
    kinematic: TrajectoryData,
    sweep: PhysicsSweepData,
) -> go.Figure:
    """建立最終形狀疊圖、逐幀接觸力與能量曲線。"""

    physics = sweep.baseline
    figure = make_subplots(
        rows=2,
        cols=2,
        specs=[[{"type": "scene", "rowspan": 2}, {"type": "xy"}], [None, {"type": "xy"}]],
        column_widths=[0.58, 0.42],
        subplot_titles=("最終軟管形狀", "抽樣接觸力／抓持誤差", "位能／動能"),
        vertical_spacing=0.12,
    )
    for name, trajectory, color, dash in (
        ("幾何約束", kinematic, "#718592", "dash"),
        ("MuJoCo cable", physics, "#18ad9c", "solid"),
    ):
        nodes = trajectory.frames[-1].hose_nodes
        figure.add_trace(
            go.Scatter3d(
                x=nodes[:, 0],
                y=nodes[:, 1],
                z=nodes[:, 2],
                mode="lines+markers",
                line={"color": color, "width": 8, "dash": dash},
                marker={"size": 3, "color": color},
                name=name,
                hovertemplate=(
                    f"{name}<br>(%{{x:.3f}}, %{{y:.3f}}, %{{z:.3f}}) m<extra></extra>"
                ),
            ),
            row=1,
            col=1,
        )
    for obstacle in physics.spec.obstacles:
        start = np.asarray(obstacle.start)
        end = np.asarray(obstacle.end)
        figure.add_trace(
            go.Scatter3d(
                x=[start[0], end[0]],
                y=[start[1], end[1]],
                z=[start[2], end[2]],
                mode="lines",
                line={"color": "#405463", "width": max(8, obstacle.radius * 260)},
                name=obstacle.name,
                hoverinfo="skip",
                showlegend=False,
            ),
            row=1,
            col=1,
        )

    time_s = [frame.time_s for frame in physics.frames]
    figure.add_trace(
        go.Scatter(
            x=time_s,
            y=[frame.maximum_contact_force_n for frame in physics.frames],
            mode="lines",
            line={"color": "#d85c4a", "width": 2.5},
            name="接觸力 (N)",
        ),
        row=1,
        col=2,
    )
    figure.add_trace(
        go.Scatter(
            x=time_s,
            y=[frame.grasp_constraint_error_m * 1000.0 for frame in physics.frames],
            mode="lines",
            line={"color": "#e89a36", "width": 2.5},
            name="抓持誤差 (mm)",
        ),
        row=1,
        col=2,
    )
    figure.add_trace(
        go.Scatter(
            x=time_s,
            y=[frame.potential_energy_j for frame in physics.frames],
            mode="lines",
            line={"color": "#2787c7", "width": 2.5},
            name="位能 (J)",
        ),
        row=2,
        col=2,
    )
    figure.add_trace(
        go.Scatter(
            x=time_s,
            y=[frame.kinetic_energy_j for frame in physics.frames],
            mode="lines",
            line={"color": "#675d9b", "width": 2.5},
            name="動能 (J)",
        ),
        row=2,
        col=2,
    )
    figure.update_scenes(
        aspectmode="data",
        xaxis_title="X (m)",
        yaxis_title="Y (m)",
        zaxis_title="Z (m)",
        camera={"eye": {"x": 1.45, "y": -1.55, "z": 1.1}},
    )
    figure.update_xaxes(title_text="時間 (s)", row=1, col=2)
    figure.update_xaxes(title_text="時間 (s)", row=2, col=2)
    figure.update_yaxes(title_text="N / mm", row=1, col=2)
    figure.update_yaxes(title_text="J", row=2, col=2)
    figure.update_layout(
        height=760,
        margin={"l": 16, "r": 16, "t": 58, "b": 36},
        paper_bgcolor="#f7fafb",
        plot_bgcolor="#f7fafb",
        legend={"orientation": "h", "y": -0.08},
    )
    return figure


def write_physics_comparison_html(
    kinematic: TrajectoryData,
    sweep: PhysicsSweepData,
    output_path: str | Path,
) -> Path:
    """輸出可離線開啟的物理比較頁面。"""

    destination = Path(output_path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    build_physics_comparison_figure(kinematic, sweep).write_html(
        destination,
        include_plotlyjs=True,
        full_html=True,
        config={"displaylogo": False, "responsive": True, "scrollZoom": True},
    )
    return destination
