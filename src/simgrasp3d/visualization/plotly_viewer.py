"""使用 Plotly 產生免伺服器的互動式 3D 學習頁面。"""

from __future__ import annotations

import numpy as np
import plotly.graph_objects as go

from simgrasp3d.geometry.sampling import PointCloud
from simgrasp3d.scene.builder import SceneData
from simgrasp3d.visualization.theme import (
    CERAMIC,
    MONO_FONT,
    SCANLINE,
    SLATE,
    TITANIUM,
    VIOLET,
    instrument_layout,
    scene_axes,
)


def _rgb(color: tuple[float, float, float]) -> str:
    channels = tuple(int(round(value * 255.0)) for value in color)
    return f"rgb({channels[0]},{channels[1]},{channels[2]})"


def _marker_style(cloud: PointCloud) -> tuple[float, float]:
    styles = {
        "environment": (1.2, 0.38),
        "object": (2.4, 0.92),
        "robot": (1.8, 0.78),
        "robot_link": (2.0, 0.82),
        "robot_joint": (2.2, 0.88),
        "gripper": (2.2, 0.90),
    }
    return styles.get(cloud.category, (2.0, 0.8))


def _cloud_trace(cloud: PointCloud) -> go.Scatter3d:
    minimum, maximum = cloud.bounds
    extent = maximum - minimum
    size, opacity = _marker_style(cloud)
    hover = (
        f"<b>{cloud.name}</b><br>"
        f"類別：{cloud.category}<br>"
        f"點數：{cloud.points.shape[0]}<br>"
        f"AABB：{extent[0]:.3f} × {extent[1]:.3f} × {extent[2]:.3f} m<br>"
        "x=%{x:.3f} m<br>y=%{y:.3f} m<br>z=%{z:.3f} m<extra></extra>"
    )
    return go.Scatter3d(
        x=cloud.points[:, 0],
        y=cloud.points[:, 1],
        z=cloud.points[:, 2],
        mode="markers",
        name=cloud.name,
        legendgroup=cloud.category,
        marker={"size": size, "color": _rgb(cloud.color), "opacity": opacity},
        hovertemplate=hover,
    )


def _segments_trace(segments: np.ndarray, name: str, color: str, width: float) -> go.Scatter3d:
    x_values: list[float | None] = []
    y_values: list[float | None] = []
    z_values: list[float | None] = []
    for start, end in segments:
        x_values.extend((float(start[0]), float(end[0]), None))
        y_values.extend((float(start[1]), float(end[1]), None))
        z_values.extend((float(start[2]), float(end[2]), None))
    return go.Scatter3d(
        x=x_values,
        y=y_values,
        z=z_values,
        mode="lines",
        name=name,
        line={"color": color, "width": width},
        hoverinfo="skip",
    )


def _frame_traces(name: str, frame: np.ndarray, scale: float = 0.09) -> list[go.Scatter3d]:
    origin = frame[:3, 3]
    colors = ("#e53935", "#43a047", "#1e88e5")
    axis_names = ("x", "y", "z")
    traces: list[go.Scatter3d] = []
    for index, (axis_name, color) in enumerate(zip(axis_names, colors, strict=True)):
        end = origin + frame[:3, index] * scale
        traces.append(
            go.Scatter3d(
                x=[origin[0], end[0]],
                y=[origin[1], end[1]],
                z=[origin[2], end[2]],
                mode="lines+text",
                name=f"{name}_{axis_name}",
                text=["", f"{name}:{axis_name}"],
                textposition="top center",
                line={"color": color, "width": 5},
                showlegend=False,
                hovertemplate=f"{name} {axis_name} 軸<extra></extra>",
            )
        )
    return traces


def build_figure(scene_data: SceneData) -> go.Figure:
    """建立可旋轉、縮放、隱藏圖層與檢查座標的 3D 圖。"""

    figure = go.Figure()
    for cloud in scene_data.point_clouds:
        figure.add_trace(_cloud_trace(cloud))

    joint_positions = scene_data.robot_state.joint_positions
    skeleton_segments = np.asarray(
        [
            [joint_positions[index], joint_positions[index + 1]]
            for index in range(len(joint_positions) - 1)
        ]
    )
    figure.add_trace(_segments_trace(skeleton_segments, "robot_skeleton", TITANIUM, 7))
    figure.add_trace(_segments_trace(scene_data.camera_segments, "camera_frustum", VIOLET, 4))

    frame_names = ["world", "robot_base", "tool", "camera"] + [
        item.name for item in scene_data.spec.objects
    ]
    for frame_name in frame_names:
        figure.add_traces(_frame_traces(frame_name, scene_data.frames[frame_name]))

    tool_position = scene_data.robot_state.tool_frame[:3, 3]
    figure.update_layout(
        **instrument_layout(
            height=720,
            margin={"l": 0, "r": 0, "t": 84, "b": 12},
            title={
                "text": (
                    f"SimGrasp3D Lab｜{scene_data.spec.name}"
                    f"<br><sup>純模擬點雲｜seed={scene_data.spec.seed}｜"
                    f"TCP=({tool_position[0]:.3f}, {tool_position[1]:.3f}, "
                    f"{tool_position[2]:.3f}) m</sup>"
                ),
                "x": 0.02,
                "xanchor": "left",
            },
        ),
        uirevision="world-camera",
        legend={
            "x": 0.01,
            "y": 0.99,
            "bgcolor": "rgba(244,247,246,0.88)",
            "bordercolor": SCANLINE,
            "borderwidth": 1,
            "font": {"size": 10},
        },
        scene={
            "uirevision": "world-camera",
            "xaxis": scene_axes("X（m）"),
            "yaxis": scene_axes("Y（m）"),
            "zaxis": scene_axes("Z（m）"),
            "aspectmode": "data",
            "camera": {"eye": {"x": 1.55, "y": -1.55, "z": 1.05}},
            "bgcolor": CERAMIC,
            "dragmode": "orbit",
        },
        annotations=[
            {
                "text": "拖曳旋轉｜滾輪縮放｜點選圖例切換圖層｜停留讀取座標與尺寸",
                "xref": "paper",
                "yref": "paper",
                "x": 0.5,
                "y": 0.01,
                "showarrow": False,
                "bgcolor": "rgba(244,247,246,0.88)",
                "bordercolor": SCANLINE,
                "borderwidth": 1,
                "font": {"family": MONO_FONT, "size": 10, "color": SLATE},
            }
        ],
    )
    return figure

