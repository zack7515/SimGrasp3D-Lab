"""建立軟管夾取、避障與搬運的互動式 3D 動畫。"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import plotly.graph_objects as go

from simgrasp3d.models.motion import PipeObstacleSpec, TrajectoryData, TrajectoryFrame
from simgrasp3d.models.specs import RobotSpec, TableSpec


def _rgb(color: tuple[float, float, float]) -> str:
    channels = tuple(int(round(value * 255.0)) for value in color)
    return f"rgb({channels[0]},{channels[1]},{channels[2]})"


def _status_color(frame: TrajectoryFrame, safe_clearance_m: float) -> str:
    if frame.collision:
        return "#d85c4a"
    if frame.minimum_clearance_m < safe_clearance_m:
        return "#e89a36"
    return "#18ad9c"


def _dynamic_traces(
    frame: TrajectoryFrame,
    trajectory: TrajectoryData,
) -> list[go.Scatter3d]:
    """建立單幀的機械臂、軟管、TCP 與夾爪代理幾何。"""

    status_color = _status_color(frame, trajectory.spec.safe_clearance_m)
    joints = frame.robot_joint_positions
    nodes = frame.hose_nodes
    tcp = frame.tcp_position
    half_opening = frame.gripper_opening_m / 2.0
    gripper_x = [tcp[0], tcp[0], None, tcp[0], tcp[0]]
    gripper_y = [
        tcp[1] - half_opening,
        tcp[1] + half_opening,
        None,
        tcp[1] - half_opening,
        tcp[1] + half_opening,
    ]
    gripper_z = [tcp[2], tcp[2], None, tcp[2] + 0.025, tcp[2] + 0.025]
    attached_label = "已附著" if frame.attached else "未附著"
    return [
        go.Scatter3d(
            x=joints[:, 0],
            y=joints[:, 1],
            z=joints[:, 2],
            mode="lines+markers",
            name="機械臂骨架",
            line={"color": "#263746", "width": 10},
            marker={"size": 5, "color": "#e4b44c"},
            hovertemplate="關節座標<br>x=%{x:.3f}<br>y=%{y:.3f}<br>z=%{z:.3f} m<extra></extra>",
        ),
        go.Scatter3d(
            x=nodes[:, 0],
            y=nodes[:, 1],
            z=nodes[:, 2],
            mode="lines+markers",
            name=trajectory.spec.hose.name,
            line={"color": _rgb(trajectory.spec.hose.color), "width": 9},
            marker={"size": 2.5, "color": status_color},
            hovertemplate="軟管節點<br>x=%{x:.3f}<br>y=%{y:.3f}<br>z=%{z:.3f} m<extra></extra>",
        ),
        go.Scatter3d(
            x=[tcp[0]],
            y=[tcp[1]],
            z=[tcp[2]],
            mode="markers+text",
            name="TCP / 狀態",
            marker={"size": 9, "color": status_color, "symbol": "diamond"},
            text=[f"{frame.phase}｜{frame.time_s:.1f}s"],
            textposition="top center",
            hovertemplate=(
                f"<b>{frame.phase}</b><br>t={frame.time_s:.2f} s<br>"
                f"狀態：{attached_label}<br>最小距離={frame.minimum_clearance_m * 1000.0:.2f} mm<br>"
                f"IK 誤差={frame.ik_position_error_m * 1000.0:.2f} mm<extra></extra>"
            ),
        ),
        go.Scatter3d(
            x=gripper_x,
            y=gripper_y,
            z=gripper_z,
            mode="lines",
            name="夾爪代理",
            line={"color": "#f4f7f8", "width": 12},
            hovertemplate=f"夾爪開口={frame.gripper_opening_m * 1000.0:.1f} mm<extra></extra>",
        ),
    ]


def _table_trace(table: TableSpec) -> go.Mesh3d:
    center = np.asarray(table.pose.xyz, dtype=np.float64)
    half_x, half_y = table.size[0] / 2.0, table.size[1] / 2.0
    z = center[2] + table.size[2] / 2.0
    return go.Mesh3d(
        x=[center[0] - half_x, center[0] + half_x, center[0] + half_x, center[0] - half_x],
        y=[center[1] - half_y, center[1] - half_y, center[1] + half_y, center[1] + half_y],
        z=[z, z, z, z],
        i=[0, 0],
        j=[1, 2],
        k=[2, 3],
        name="工作桌面",
        color="#d8e1e4",
        opacity=0.52,
        hoverinfo="skip",
    )


def _cylinder_trace(obstacle: PipeObstacleSpec, sides: int = 24) -> go.Mesh3d:
    """以實際公尺半徑建立固定管路圓柱網格。"""

    start = np.asarray(obstacle.start, dtype=np.float64)
    end = np.asarray(obstacle.end, dtype=np.float64)
    axis = end - start
    axis /= np.linalg.norm(axis)
    reference = np.asarray([0.0, 0.0, 1.0])
    if abs(float(np.dot(axis, reference))) > 0.92:
        reference = np.asarray([0.0, 1.0, 0.0])
    first_basis = np.cross(axis, reference)
    first_basis /= np.linalg.norm(first_basis)
    second_basis = np.cross(axis, first_basis)
    angles = np.linspace(0.0, 2.0 * np.pi, sides, endpoint=False)
    ring_offsets = obstacle.radius * (
        np.cos(angles)[:, None] * first_basis
        + np.sin(angles)[:, None] * second_basis
    )
    vertices = np.vstack((start + ring_offsets, end + ring_offsets, start, end))
    start_center = 2 * sides
    end_center = 2 * sides + 1
    face_i: list[int] = []
    face_j: list[int] = []
    face_k: list[int] = []
    for index in range(sides):
        following = (index + 1) % sides
        face_i.extend((index, index, start_center, end_center))
        face_j.extend((following, sides + following, following, sides + index))
        face_k.extend((sides + following, sides + index, index, sides + following))
    return go.Mesh3d(
        x=vertices[:, 0],
        y=vertices[:, 1],
        z=vertices[:, 2],
        i=face_i,
        j=face_j,
        k=face_k,
        name=obstacle.name,
        color=_rgb(obstacle.color),
        opacity=0.92,
        flatshading=True,
        hovertemplate=(
            f"<b>{obstacle.name}</b><br>半徑={obstacle.radius * 1000.0:.1f} mm"
            "<extra></extra>"
        ),
    )


def build_motion_figure(
    trajectory: TrajectoryData,
    robot: RobotSpec,
    table: TableSpec,
) -> go.Figure:
    """建立含播放鍵、逐幀滑桿與安全狀態的 3D 動畫。"""

    first = trajectory.frames[0]
    figure = go.Figure(data=_dynamic_traces(first, trajectory))
    figure.add_trace(_table_trace(table))

    for obstacle in trajectory.spec.obstacles:
        figure.add_trace(_cylinder_trace(obstacle))

    keyframe_positions = np.asarray(
        [keyframe.tcp_position for keyframe in trajectory.spec.keyframes],
        dtype=np.float64,
    )
    figure.add_trace(
        go.Scatter3d(
            x=keyframe_positions[:, 0],
            y=keyframe_positions[:, 1],
            z=keyframe_positions[:, 2],
            mode="lines+markers",
            name="TCP 規劃路徑",
            line={"color": "#718592", "width": 4, "dash": "dash"},
            marker={"size": 3, "color": "#718592"},
            hovertemplate="規劃節點<br>x=%{x:.3f}<br>y=%{y:.3f}<br>z=%{z:.3f} m<extra></extra>",
        )
    )

    target = np.asarray(trajectory.spec.target_position, dtype=np.float64)
    angles = np.linspace(0.0, 2.0 * np.pi, 65)
    figure.add_trace(
        go.Scatter3d(
            x=target[0] + trajectory.spec.target_radius_m * np.cos(angles),
            y=target[1] + trajectory.spec.target_radius_m * np.sin(angles),
            z=np.full_like(angles, table.pose.xyz[2] + table.size[2] / 2.0 + 0.003),
            mode="lines",
            name="放置目標區",
            line={"color": "#18ad9c", "width": 6},
            hovertemplate="軟管放置目標區<extra></extra>",
        )
    )

    frame_duration_ms = int(round(1000.0 / trajectory.spec.frame_rate_hz))
    figure.frames = tuple(
        go.Frame(
            name=f"{index:04d}",
            data=_dynamic_traces(frame, trajectory),
            traces=[0, 1, 2, 3],
        )
        for index, frame in enumerate(trajectory.frames)
    )
    slider_steps = []
    previous_phase = ""
    for index, frame in enumerate(trajectory.frames):
        label = f"{frame.time_s:.1f}s"
        if frame.phase != previous_phase:
            label = f"{frame.time_s:.1f}s · {frame.phase}"
            previous_phase = frame.phase
        slider_steps.append(
            {
                "args": [
                    [f"{index:04d}"],
                    {
                        "frame": {"duration": 0, "redraw": True},
                        "mode": "immediate",
                        "transition": {"duration": 0},
                    },
                ],
                "label": label,
                "method": "animate",
            }
        )

    metrics = trajectory.metrics
    figure.update_layout(
        template="plotly_white",
        height=720,
        margin={"l": 0, "r": 0, "t": 45, "b": 100},
        paper_bgcolor="#f7fafb",
        plot_bgcolor="#f7fafb",
        legend={"x": 0.01, "y": 0.99, "bgcolor": "rgba(255,255,255,0.80)"},
        scene={
            "xaxis": {"title": "X（m）", "range": [-0.78, 0.76]},
            "yaxis": {"title": "Y（m）", "range": [-0.58, 0.58]},
            "zaxis": {"title": "Z（m）", "range": [0.0, 0.86]},
            "aspectmode": "manual",
            "aspectratio": {"x": 1.55, "y": 1.15, "z": 0.86},
            "camera": {"eye": {"x": 1.45, "y": -1.55, "z": 1.05}},
            "bgcolor": "#f2f6f7",
        },
        sliders=[
            {
                "active": 0,
                "currentvalue": {"prefix": "FRAME / ", "font": {"size": 12, "color": "#263746"}},
                "pad": {"t": 45, "b": 5},
                "steps": slider_steps,
            }
        ],
        updatemenus=[
            {
                "type": "buttons",
                "direction": "left",
                "x": 0.0,
                "y": -0.09,
                "showactive": False,
                "buttons": [
                    {
                        "label": "▶ 播放",
                        "method": "animate",
                        "args": [
                            None,
                            {
                                "frame": {"duration": frame_duration_ms, "redraw": True},
                                "fromcurrent": True,
                                "transition": {"duration": 0},
                            },
                        ],
                    },
                    {
                        "label": "Ⅱ 暫停",
                        "method": "animate",
                        "args": [
                            [None],
                            {
                                "frame": {"duration": 0, "redraw": False},
                                "mode": "immediate",
                                "transition": {"duration": 0},
                            },
                        ],
                    },
                ],
            }
        ],
        annotations=[
            {
                "text": (
                    f"IK max {metrics['maximum_ik_error_m'] * 1000.0:.2f} mm ｜ "
                    f"軟管長度 max Δ {metrics['maximum_hose_length_error_ratio'] * 100.0:.2f}% ｜ "
                    f"碰撞幀 {metrics['collision_frame_count']}"
                ),
                "xref": "paper",
                "yref": "paper",
                "x": 1.0,
                "y": -0.08,
                "xanchor": "right",
                "showarrow": False,
                "font": {"family": "monospace", "size": 11, "color": "#536777"},
            }
        ],
    )
    return figure


def write_motion_html(
    trajectory: TrajectoryData,
    robot: RobotSpec,
    table: TableSpec,
    output_path: str | Path,
) -> Path:
    """輸出可離線播放的自包含軟管動作頁面。"""

    destination = Path(output_path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    build_motion_figure(trajectory, robot, table).write_html(
        destination,
        include_plotlyjs=True,
        full_html=True,
        config={"displaylogo": False, "responsive": True, "scrollZoom": True},
    )
    return destination
