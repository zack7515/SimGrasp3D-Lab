"""建立桌面分割、OBB、法向與抓取候選的互動式 3D 圖。"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import plotly.graph_objects as go

from simgrasp3d.models.perception import BoundingBox3D, PerceptionResult
from simgrasp3d.sensors.rgbd import RGBDFrame


_BOX_EDGES = (
    (0, 1),
    (1, 2),
    (2, 3),
    (3, 0),
    (4, 5),
    (5, 6),
    (6, 7),
    (7, 4),
    (0, 4),
    (1, 5),
    (2, 6),
    (3, 7),
)


def _line_segments(segments: list[np.ndarray]) -> tuple[list[float | None], ...]:
    x_values: list[float | None] = []
    y_values: list[float | None] = []
    z_values: list[float | None] = []
    for segment in segments:
        x_values.extend((float(segment[0, 0]), float(segment[1, 0]), None))
        y_values.extend((float(segment[0, 1]), float(segment[1, 1]), None))
        z_values.extend((float(segment[0, 2]), float(segment[1, 2]), None))
    return x_values, y_values, z_values


def _box_segments(box: BoundingBox3D) -> list[np.ndarray]:
    corners = box.corners()
    return [np.asarray([corners[first], corners[second]]) for first, second in _BOX_EDGES]


def build_perception_figure(
    frame: RGBDFrame,
    result: PerceptionResult,
) -> go.Figure:
    """建立 observation 點雲與所有幾何推論的疊圖。"""

    points = frame.world_points()
    figure = go.Figure()
    figure.add_trace(
        go.Scatter3d(
            x=points[:, 0],
            y=points[:, 1],
            z=points[:, 2],
            mode="markers",
            marker={"size": 2, "color": "#9aa9b2", "opacity": 0.22},
            name="Observation",
            hoverinfo="skip",
        )
    )
    palette = ("#2787c7", "#e8902e", "#38a169")
    for object_index, geometry in enumerate(result.objects):
        color = palette[object_index % len(palette)]
        figure.add_trace(
            go.Scatter3d(
                x=geometry.points[:, 0],
                y=geometry.points[:, 1],
                z=geometry.points[:, 2],
                mode="markers",
                marker={"size": 3.5, "color": color, "opacity": 0.9},
                name=geometry.name,
                text=[f"{geometry.name}<br>可見點 {len(geometry.points)}"]
                * len(geometry.points),
                hovertemplate="%{text}<br>(%{x:.3f}, %{y:.3f}, %{z:.3f}) m<extra></extra>",
            )
        )
        box_x, box_y, box_z = _line_segments(_box_segments(geometry.obb))
        figure.add_trace(
            go.Scatter3d(
                x=box_x,
                y=box_y,
                z=box_z,
                mode="lines",
                line={"color": color, "width": 5},
                name=f"{geometry.name} OBB",
                hoverinfo="skip",
            )
        )
        normal_segments = [
            np.asarray([point, point + normal * 0.018])
            for point, normal in zip(
                geometry.normal_points[::4],
                geometry.normals[::4],
                strict=True,
            )
        ]
        normal_x, normal_y, normal_z = _line_segments(normal_segments)
        figure.add_trace(
            go.Scatter3d(
                x=normal_x,
                y=normal_y,
                z=normal_z,
                mode="lines",
                line={"color": "#675d9b", "width": 2},
                name=f"{geometry.name} normals",
                hoverinfo="skip",
                visible="legendonly",
            )
        )

    for index, candidate in enumerate(result.grasp_candidates):
        status_color = "#18ad9c" if candidate.geometry_feasible else "#d85c4a"
        half_width = candidate.required_opening_m / 2.0
        closing_segment = np.asarray(
            [
                candidate.tcp_position - candidate.closing_axis * half_width,
                candidate.tcp_position + candidate.closing_axis * half_width,
            ]
        )
        approach_segment = np.asarray(
            [candidate.pregrasp_position, candidate.tcp_position]
        )
        grasp_x, grasp_y, grasp_z = _line_segments(
            [approach_segment, closing_segment]
        )
        figure.add_trace(
            go.Scatter3d(
                x=grasp_x,
                y=grasp_y,
                z=grasp_z,
                mode="lines",
                line={"color": status_color, "width": 7},
                name=f"G{index + 1} {candidate.object_name}",
                text=[
                    f"score={candidate.score:.3f}<br>opening={candidate.required_opening_m * 1000.0:.1f} mm"
                ]
                * len(grasp_x),
                hovertemplate="%{text}<extra></extra>",
            )
        )

    figure.update_layout(
        title={"text": "RGB-D 幾何與抓取候選", "x": 0.02, "xanchor": "left"},
        scene={
            "aspectmode": "data",
            "xaxis_title": "X (m)",
            "yaxis_title": "Y (m)",
            "zaxis_title": "Z (m)",
            "camera": {"eye": {"x": 1.45, "y": -1.65, "z": 1.15}},
        },
        legend={"orientation": "h", "y": -0.08},
        margin={"l": 0, "r": 0, "t": 48, "b": 28},
        paper_bgcolor="#f7fafb",
        plot_bgcolor="#f7fafb",
        height=680,
    )
    return figure


def write_perception_html(
    frame: RGBDFrame,
    result: PerceptionResult,
    output_path: str | Path,
) -> Path:
    """輸出不依賴 CDN 的 perception 互動式 HTML。"""

    destination = Path(output_path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    build_perception_figure(frame, result).write_html(
        destination,
        include_plotlyjs=True,
        full_html=True,
        config={"displaylogo": False, "responsive": True, "scrollZoom": True},
    )
    return destination
