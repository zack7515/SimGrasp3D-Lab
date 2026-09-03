"""建立軟管夾取、避障與搬運的互動式 3D 動畫。"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import plotly.graph_objects as go

from simgrasp3d.models.motion import PipeObstacleSpec, TrajectoryData, TrajectoryFrame
from simgrasp3d.models.specs import RobotSpec, TableSpec
from simgrasp3d.robot.collision import build_robot_capsules
from simgrasp3d.visualization.assets import write_plotly_asset
from simgrasp3d.visualization.theme import (
    AMBER,
    CERAMIC,
    FAULT,
    LASER,
    LASER_DARK,
    MONO_FONT,
    SCANLINE,
    SLATE,
    TITANIUM,
    VACUUM,
    instrument_layout,
    scene_axes,
)


def _rgb(color: tuple[float, float, float]) -> str:
    channels = tuple(int(round(value * 255.0)) for value in color)
    return f"rgb({channels[0]},{channels[1]},{channels[2]})"


def _status_color(frame: TrajectoryFrame, safe_clearance_m: float) -> str:
    if frame.collision:
        return FAULT
    if frame.minimum_clearance_m < safe_clearance_m:
        return AMBER
    return LASER


def _dynamic_traces(
    frame: TrajectoryFrame,
    trajectory: TrajectoryData,
    robot: RobotSpec,
    frame_index: int,
) -> list[go.Scatter3d]:
    """建立單幀的機械臂、軟管、TCP、夾取點與歷史軌跡。"""

    status_color = _status_color(frame, trajectory.spec.safe_clearance_m)
    joints = frame.robot_joint_positions
    nodes = frame.hose_nodes
    tcp = frame.tcp_position
    gripper_parts = [
        part
        for part in build_robot_capsules(
            robot,
            frame.robot_joint_positions,
            frame.tool_frame,
            frame.gripper_opening_m,
        )
        if part.category == "gripper"
    ]
    gripper_x: list[float | None] = []
    gripper_y: list[float | None] = []
    gripper_z: list[float | None] = []
    for part in gripper_parts:
        gripper_x.extend((float(part.start[0]), float(part.end[0]), None))
        gripper_y.extend((float(part.start[1]), float(part.end[1]), None))
        gripper_z.extend((float(part.start[2]), float(part.end[2]), None))
    attached_label = "已附著" if frame.attached else "未附著"
    hose_status_color = AMBER if frame.hose_clearance_m <= 0.001 else _rgb(
        trajectory.spec.hose.color
    )
    grasp_node = nodes[trajectory.spec.hose.grasp_node_index]
    history = np.asarray(
        [item.tcp_position for item in trajectory.frames[: frame_index + 1]],
        dtype=np.float64,
    )
    physics_detail = ""
    if trajectory.physics_engine is not None:
        physics_detail = (
            f"<br>抽樣接觸力={frame.maximum_contact_force_n:.2f} N"
            f"<br>抓持誤差={frame.grasp_constraint_error_m * 1000.0:.2f} mm"
        )
    return [
        go.Scatter3d(
            x=joints[:, 0],
            y=joints[:, 1],
            z=joints[:, 2],
            mode="lines+markers",
            name="機械臂骨架",
            line={"color": TITANIUM, "width": 10},
            marker={"size": 5, "color": AMBER},
            hovertemplate="關節座標<br>x=%{x:.3f}<br>y=%{y:.3f}<br>z=%{z:.3f} m<extra></extra>",
        ),
        go.Scatter3d(
            x=nodes[:, 0],
            y=nodes[:, 1],
            z=nodes[:, 2],
            mode="lines+markers",
            name=trajectory.spec.hose.name,
            line={"color": _rgb(trajectory.spec.hose.color), "width": 9},
            marker={"size": 2.5, "color": hose_status_color},
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
                f"狀態：{attached_label}<br>機器人距離={frame.minimum_clearance_m * 1000.0:.2f} mm<br>"
                f"最近：{frame.closest_collision_pair}<br>"
                f"IK 位置={frame.ik_position_error_m * 1000.0:.2f} mm<br>"
                f"IK 姿態={frame.ik_orientation_error_deg:.2f}°"
                f"{physics_detail}<extra></extra>"
            ),
        ),
        go.Scatter3d(
            x=gripper_x,
            y=gripper_y,
            z=gripper_z,
            mode="lines",
            name="夾爪代理",
            line={"color": "#FFFFFF", "width": 12},
            hovertemplate=f"夾爪開口={frame.gripper_opening_m * 1000.0:.1f} mm<extra></extra>",
        ),
        go.Scatter3d(
            x=[grasp_node[0]],
            y=[grasp_node[1]],
            z=[grasp_node[2]],
            mode="markers",
            name="軟管夾取節點",
            marker={
                "size": 8,
                "color": LASER if frame.attached else AMBER,
                "line": {"color": "#FFFFFF", "width": 2},
            },
            hovertemplate=(
                f"夾取節點 {trajectory.spec.hose.grasp_node_index}<br>"
                f"狀態：{attached_label}<extra></extra>"
            ),
        ),
        go.Scatter3d(
            x=history[:, 0],
            y=history[:, 1],
            z=history[:, 2],
            mode="lines",
            name="TCP 已走路徑",
            line={"color": LASER_DARK, "width": 4},
            opacity=0.72,
            hoverinfo="skip",
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
        color=SCANLINE,
        opacity=0.46,
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
    figure = go.Figure(data=_dynamic_traces(first, trajectory, robot, 0))
    dynamic_trace_count = len(figure.data)
    figure.add_trace(_table_trace(table))

    for obstacle in trajectory.spec.obstacles:
        figure.add_trace(_cylinder_trace(obstacle))

    keyframe_positions = np.asarray(
        [keyframe.tcp_position for keyframe in trajectory.planned_keyframes],
        dtype=np.float64,
    )
    figure.add_trace(
        go.Scatter3d(
            x=keyframe_positions[:, 0],
            y=keyframe_positions[:, 1],
            z=keyframe_positions[:, 2],
            mode="lines+markers",
            name="TCP 規劃路徑",
            line={"color": SLATE, "width": 4, "dash": "dash"},
            marker={
                "size": 4,
                "color": [
                    AMBER if keyframe.generated else SLATE
                    for keyframe in trajectory.planned_keyframes
                ],
            },
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
            line={"color": LASER, "width": 6},
            hovertemplate="軟管放置目標區<extra></extra>",
        )
    )

    frame_duration_ms = int(round(1000.0 / trajectory.spec.frame_rate_hz))
    figure.frames = tuple(
        go.Frame(
            name=f"{index:04d}",
            data=_dynamic_traces(frame, trajectory, robot, index),
            traces=list(range(dynamic_trace_count)),
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
        **instrument_layout(
            height=720,
            margin={"l": 0, "r": 0, "t": 38, "b": 108},
        ),
        uirevision="motion-camera",
        legend={
            "x": 0.01,
            "y": 0.99,
            "bgcolor": "rgba(244,247,246,0.88)",
            "bordercolor": SCANLINE,
            "borderwidth": 1,
            "font": {"size": 10},
        },
        scene={
            "uirevision": "motion-camera",
            "xaxis": scene_axes("X（m）", [-0.78, 0.76]),
            "yaxis": scene_axes("Y（m）", [-0.58, 0.58]),
            "zaxis": scene_axes("Z（m）", [0.0, 0.86]),
            "aspectmode": "manual",
            "aspectratio": {"x": 1.55, "y": 1.15, "z": 0.86},
            "camera": {"eye": {"x": 1.45, "y": -1.55, "z": 1.05}},
            "bgcolor": CERAMIC,
            "dragmode": "orbit",
        },
        sliders=[
            {
                "active": 0,
                "currentvalue": {
                    "prefix": "TIME / ",
                    "font": {"family": MONO_FONT, "size": 11, "color": TITANIUM},
                },
                "activebgcolor": LASER,
                "bgcolor": SCANLINE,
                "bordercolor": "#A7B7B3",
                "font": {"family": MONO_FONT, "size": 9, "color": SLATE},
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
                "bgcolor": VACUUM,
                "bordercolor": VACUUM,
                "font": {"family": MONO_FONT, "size": 10, "color": "#FFFFFF"},
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
                    {
                        "label": "↺ 從頭",
                        "method": "animate",
                        "args": [
                            [item.name for item in figure.frames],
                            {
                                "frame": {
                                    "duration": frame_duration_ms,
                                    "redraw": True,
                                },
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
                    f"姿態 {metrics['maximum_ik_orientation_error_deg']:.2f}° ｜ "
                    f"AUTO WP {metrics['inserted_waypoint_count']} ｜ "
                    f"機器人警示 {metrics['unsafe_clearance_frame_count']}"
                ),
                "xref": "paper",
                "yref": "paper",
                "x": 1.0,
                "y": -0.08,
                "xanchor": "right",
                "showarrow": False,
                "font": {"family": MONO_FONT, "size": 10, "color": SLATE},
                "bgcolor": "rgba(244,247,246,0.82)",
                "bordercolor": SCANLINE,
                "borderwidth": 1,
            }
        ],
    )
    return figure


def write_motion_html(
    trajectory: TrajectoryData,
    robot: RobotSpec,
    table: TableSpec,
    output_path: str | Path,
    asset_root: str | Path | None = None,
) -> Path:
    """輸出可離線播放的自包含軟管動作頁面。"""

    destination = Path(output_path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    build_motion_figure(trajectory, robot, table).write_html(
        destination,
        include_plotlyjs=write_plotly_asset(destination, asset_root),
        full_html=True,
        config={"displaylogo": False, "responsive": True, "scrollZoom": True},
    )
    return destination
