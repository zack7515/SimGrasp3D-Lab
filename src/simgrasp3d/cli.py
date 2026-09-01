"""建立、匯出並視覺化 3D 學習場景的命令列入口。"""

from __future__ import annotations

import argparse
import webbrowser
from pathlib import Path

from simgrasp3d.io.rgbd_frame import export_rgbd_simulation
from simgrasp3d.io.point_cloud import export_scene_point_clouds
from simgrasp3d.scene.builder import build_scene, load_scene_spec
from simgrasp3d.sensors.rgbd import simulate_rgbd
from simgrasp3d.visualization.plotly_viewer import write_scene_html
from simgrasp3d.visualization.rgbd_viewer import write_rgbd_comparison_html


def build_parser() -> argparse.ArgumentParser:
    """建立命令列參數解析器。"""

    parser = argparse.ArgumentParser(
        description="產生 SimGrasp3D Lab 的互動式 3D 點雲學習場景。"
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=Path("configs/scenes/tabletop_demo.json"),
        help="場景 JSON 設定檔",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("outputs/tabletop_scene.html"),
        help="互動式 HTML 輸出路徑",
    )
    parser.add_argument(
        "--point-cloud-dir",
        type=Path,
        default=Path("outputs/point_clouds"),
        help="PLY 點雲輸出目錄",
    )
    parser.add_argument(
        "--sensor-output-dir",
        type=Path,
        default=Path("outputs/sensor"),
        help="RGB-D frame、可見點雲、比較頁面與指標輸出目錄",
    )
    parser.add_argument(
        "--no-export-point-clouds",
        action="store_true",
        help="只建立 HTML，不匯出 PLY",
    )
    parser.add_argument(
        "--no-simulate-rgbd",
        action="store_true",
        help="略過 RGB-D 投影與感測誤差模擬",
    )
    parser.add_argument(
        "--open",
        action="store_true",
        help="完成後使用預設瀏覽器開啟 HTML",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    """執行場景建立流程並輸出摘要。"""

    args = build_parser().parse_args(argv)
    spec = load_scene_spec(args.config)
    scene_data = build_scene(spec)
    html_path = write_scene_html(scene_data, args.output).resolve()

    exported_count = 0
    if not args.no_export_point_clouds:
        exported_count = len(export_scene_point_clouds(args.point_cloud_dir, scene_data.point_clouds))

    total_points = sum(cloud.points.shape[0] for cloud in scene_data.point_clouds)
    tool_position = scene_data.robot_state.tool_frame[:3, 3]
    print(f"場景：{spec.name}")
    print(f"單位：{spec.units}｜seed：{spec.seed}")
    print(f"實體：{len(scene_data.point_clouds)}｜總點數：{total_points}")
    print(
        "TCP 世界座標："
        f"({tool_position[0]:.4f}, {tool_position[1]:.4f}, {tool_position[2]:.4f}) m"
    )
    print(f"互動式視覺化：{html_path}")
    if exported_count:
        print(f"PLY 點雲：{args.point_cloud_dir.resolve()}（{exported_count} 個檔案）")

    comparison_path: Path | None = None
    if not args.no_simulate_rgbd:
        sensor_result = simulate_rgbd(scene_data)
        sensor_paths = export_rgbd_simulation(args.sensor_output_dir, sensor_result)
        comparison_path = write_rgbd_comparison_html(
            sensor_result,
            args.sensor_output_dir / "rgbd_comparison.html",
        ).resolve()
        metrics = sensor_result.metrics
        print(
            "RGB-D 誤差："
            f"MAE={metrics['depth_mae_m'] * 1000.0:.2f} mm｜"
            f"RMSE={metrics['depth_rmse_m'] * 1000.0:.2f} mm｜"
            f"有效點={metrics['observation_valid_pixels']}"
        )
        print(f"RGB-D frame：{sensor_paths['observation_frame'].resolve()}")
        print(f"RGB-D 比較：{comparison_path}")

    if args.open:
        webbrowser.open(html_path.as_uri())
        if comparison_path is not None:
            webbrowser.open(comparison_path.as_uri())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
