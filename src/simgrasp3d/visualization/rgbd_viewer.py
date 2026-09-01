"""建立 RGB-D ground truth 與觀測誤差的互動式比較頁面。"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import plotly.graph_objects as go
from plotly.subplots import make_subplots

from simgrasp3d.sensors.rgbd import RGBDSimulationResult


def _display_depth(depth_m: np.ndarray) -> np.ndarray:
    """將無效深度改成 NaN，避免熱圖把零值當成近距離。"""

    result = depth_m.astype(np.float64, copy=True)
    result[result <= 0.0] = np.nan
    return result


def build_rgbd_comparison_figure(result: RGBDSimulationResult) -> go.Figure:
    """建立 RGB、理想深度、觀測深度與絕對誤差四格圖。"""

    ground_truth_depth = _display_depth(result.ground_truth.depth_m)
    observation_depth = _display_depth(result.observation.depth_m)
    common = result.ground_truth.valid_mask & result.observation.valid_mask
    absolute_error = np.full(result.ground_truth.shape, np.nan, dtype=np.float64)
    absolute_error[common] = np.abs(
        result.observation.depth_m[common] - result.ground_truth.depth_m[common]
    )
    valid_depth = np.concatenate(
        (
            ground_truth_depth[np.isfinite(ground_truth_depth)],
            observation_depth[np.isfinite(observation_depth)],
        )
    )
    depth_min = float(valid_depth.min()) if valid_depth.size else 0.0
    depth_max = float(valid_depth.max()) if valid_depth.size else 1.0

    figure = make_subplots(
        rows=2,
        cols=2,
        subplot_titles=("Ground truth 深度", "Observation 深度", "絕對深度誤差", "Observation RGB"),
        horizontal_spacing=0.08,
        vertical_spacing=0.12,
    )
    heatmap_options = {
        "colorscale": "Turbo",
        "zmin": depth_min,
        "zmax": depth_max,
        "colorbar": {"title": "m", "len": 0.40, "thickness": 12},
        "hovertemplate": "u=%{x}<br>v=%{y}<br>depth=%{z:.4f} m<extra></extra>",
    }
    figure.add_trace(
        go.Heatmap(z=ground_truth_depth, **heatmap_options),
        row=1,
        col=1,
    )
    observation_options = dict(heatmap_options)
    observation_options["showscale"] = False
    figure.add_trace(
        go.Heatmap(z=observation_depth, **observation_options),
        row=1,
        col=2,
    )
    figure.add_trace(
        go.Heatmap(
            z=absolute_error,
            colorscale="Magma",
            colorbar={"title": "m", "len": 0.40, "y": 0.22, "thickness": 12},
            hovertemplate="u=%{x}<br>v=%{y}<br>|error|=%{z:.4f} m<extra></extra>",
        ),
        row=2,
        col=1,
    )
    figure.add_trace(go.Image(z=result.observation.rgb), row=2, col=2)
    metrics = result.metrics
    figure.update_layout(
        title={
            "text": (
                "SimGrasp3D RGB-D 感測模擬"
                f"<br><sup>MAE={metrics['depth_mae_m'] * 1000.0:.2f} mm｜"
                f"RMSE={metrics['depth_rmse_m'] * 1000.0:.2f} mm｜"
                f"有效觀測={metrics['observation_valid_pixels']} pixels</sup>"
            ),
            "x": 0.5,
        },
        template="plotly_white",
        height=820,
        margin={"l": 35, "r": 80, "t": 100, "b": 35},
    )
    figure.update_yaxes(autorange="reversed", scaleanchor="x", scaleratio=1, row=1, col=1)
    figure.update_yaxes(autorange="reversed", scaleanchor="x2", scaleratio=1, row=1, col=2)
    figure.update_yaxes(autorange="reversed", scaleanchor="x3", scaleratio=1, row=2, col=1)
    figure.update_yaxes(autorange="reversed", scaleanchor="x4", scaleratio=1, row=2, col=2)
    return figure


def write_rgbd_comparison_html(
    result: RGBDSimulationResult,
    output_path: str | Path,
) -> Path:
    """將 RGB-D 比較圖輸出為自包含 HTML。"""

    destination = Path(output_path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    build_rgbd_comparison_figure(result).write_html(
        destination,
        include_plotlyjs=True,
        full_html=True,
        config={"displaylogo": False, "responsive": True},
    )
    return destination
