"""建立、匯出並視覺化 3D 學習場景的命令列入口。"""

from __future__ import annotations

import argparse
import webbrowser
from pathlib import Path

from simgrasp3d.integration import build_fail_closed_replay, load_integration_spec
from simgrasp3d.io.integration import export_replay_result
from simgrasp3d.io.perception import export_perception_result
from simgrasp3d.io.physics import export_physics_sweep
from simgrasp3d.io.rgbd_frame import export_rgbd_simulation
from simgrasp3d.io.point_cloud import export_scene_point_clouds
from simgrasp3d.io.trajectory import export_trajectory
from simgrasp3d.scene.builder import build_scene, load_scene_spec
from simgrasp3d.perception import analyze_rgbd_geometry, load_perception_spec
from simgrasp3d.robot.description import export_robot_description
from simgrasp3d.sensors.rgbd import simulate_rgbd
from simgrasp3d.simulation.hose_motion import load_hose_motion_spec, simulate_hose_motion
from simgrasp3d.simulation.mujoco_hose import (
    load_mujoco_hose_spec,
    simulate_physics_sweep,
)
from simgrasp3d.visualization.motion_viewer import write_motion_html
from simgrasp3d.visualization.perception_viewer import write_perception_html
from simgrasp3d.visualization.physics_viewer import write_physics_comparison_html
from simgrasp3d.visualization.plotly_viewer import write_scene_html
from simgrasp3d.visualization.rgbd_viewer import write_rgbd_comparison_html
from simgrasp3d.visualization.simulation_report import write_simulation_report


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
        "--report-output",
        type=Path,
        default=Path("outputs/simulation_report.html"),
        help="原始場景、感測結果與連續動作的單頁報告路徑",
    )
    parser.add_argument(
        "--motion-config",
        type=Path,
        default=Path("configs/motions/hose_extraction_demo.json"),
        help="軟管連續動作 JSON 設定檔",
    )
    parser.add_argument(
        "--motion-output",
        type=Path,
        default=Path("outputs/hose_motion.html"),
        help="可離線播放的軟管連續動作 HTML 路徑",
    )
    parser.add_argument(
        "--motion-output-dir",
        type=Path,
        default=Path("outputs/motion"),
        help="逐幀軌跡 NPZ 與動作指標輸出目錄",
    )
    parser.add_argument(
        "--physics-config",
        type=Path,
        default=Path("configs/physics/hose_mujoco_baseline.json"),
        help="MuJoCo cable 與敏感度 JSON 設定檔",
    )
    parser.add_argument(
        "--physics-output-dir",
        type=Path,
        default=Path("outputs/physics"),
        help="MuJoCo 軌跡、敏感度與比較頁面輸出目錄",
    )
    parser.add_argument(
        "--perception-config",
        type=Path,
        default=Path("configs/perception/rgbd_geometry_baseline.json"),
        help="桌面、OBB、法向與抓取候選 JSON 設定檔",
    )
    parser.add_argument(
        "--perception-output-dir",
        type=Path,
        default=Path("outputs/perception"),
        help="感知幾何、物件點雲與互動頁面輸出目錄",
    )
    parser.add_argument(
        "--integration-config",
        type=Path,
        default=Path("configs/integration/fail_closed_baseline.json"),
        help="fail-closed 安全閘門 JSON 設定檔",
    )
    parser.add_argument(
        "--integration-output-dir",
        type=Path,
        default=Path("outputs/integration"),
        help="URDF、SRDF、JSONL 重播與安全摘要輸出目錄",
    )
    parser.add_argument(
        "--no-export-point-clouds",
        action="store_true",
        help="不匯出完整場景 PLY；RGB-D 感測產物不受影響",
    )
    parser.add_argument(
        "--no-simulate-rgbd",
        action="store_true",
        help="略過 RGB-D 投影與感測誤差模擬",
    )
    parser.add_argument(
        "--no-simulate-motion",
        action="store_true",
        help="略過軟管夾取、避障與搬運時間序列",
    )
    parser.add_argument(
        "--no-simulate-physics",
        action="store_true",
        help="略過 MuJoCo cable baseline 與參數敏感度",
    )
    parser.add_argument(
        "--no-analyze-perception",
        action="store_true",
        help="略過桌面、物件幾何與抓取候選分析",
    )
    parser.add_argument(
        "--no-build-replay",
        action="store_true",
        help="略過 URDF/SRDF 與 fail-closed 控制重播",
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

    motion_path: Path | None = None
    motion_trajectory = None
    if not args.no_simulate_motion:
        motion_spec = load_hose_motion_spec(args.motion_config)
        motion_trajectory = simulate_hose_motion(motion_spec, spec.robot)
        motion_paths = export_trajectory(args.motion_output_dir, motion_trajectory)
        motion_path = write_motion_html(
            motion_trajectory,
            spec.robot,
            spec.table,
            args.motion_output,
        ).resolve()
        motion_metrics = motion_trajectory.metrics
        print(
            "連續動作："
            f"{motion_metrics['frame_count']} 幀 / {motion_metrics['duration_s']:.1f} s｜"
            f"IK={motion_metrics['maximum_ik_error_m'] * 1000.0:.2f} mm / "
            f"{motion_metrics['maximum_ik_orientation_error_deg']:.2f}°｜"
            f"自動 waypoint={motion_metrics['inserted_waypoint_count']}｜"
            f"機器人警示/碰撞={motion_metrics['unsafe_clearance_frame_count']}/"
            f"{motion_metrics['collision_frame_count']}｜"
            f"軟管接觸={motion_metrics['hose_contact_frame_count']}"
        )
        print(f"軟管動畫：{motion_path}")
        print(f"逐幀軌跡：{motion_paths['trajectory'].resolve()}")

    sensor_result = None
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

    physics_sweep = None
    if motion_trajectory is not None and not args.no_simulate_physics:
        physics_spec = load_mujoco_hose_spec(args.physics_config)
        physics_sweep = simulate_physics_sweep(motion_trajectory, physics_spec)
        physics_paths = export_physics_sweep(args.physics_output_dir, physics_sweep)
        physics_motion_path = write_motion_html(
            physics_sweep.baseline,
            spec.robot,
            spec.table,
            args.physics_output_dir / "hose_physics.html",
        ).resolve()
        physics_comparison_path = write_physics_comparison_html(
            motion_trajectory,
            physics_sweep,
            args.physics_output_dir / "comparison.html",
        ).resolve()
        physics_metrics = physics_sweep.baseline.metrics
        print(
            "MuJoCo 軟管："
            f"{physics_metrics['physics_step_count']} steps｜"
            f"接觸力={physics_metrics['maximum_contact_force_n']:.2f} N｜"
            f"抓持誤差={physics_metrics['maximum_grasp_constraint_error_m'] * 1000.0:.2f} mm｜"
            f"敏感度案例={len(physics_sweep.cases)}"
        )
        print(f"物理動畫：{physics_motion_path}")
        print(f"物理比較：{physics_comparison_path}")
        print(f"物理資料：{physics_paths['trajectory'].resolve()}")

    perception_result = None
    if sensor_result is not None and not args.no_analyze_perception:
        perception_spec = load_perception_spec(args.perception_config)
        table_top_z = spec.table.pose.xyz[2] + spec.table.size[2] / 2.0
        perception_result = analyze_rgbd_geometry(
            sensor_result.observation,
            tuple(item.name for item in spec.objects),
            perception_spec,
            table_top_z,
        )
        perception_paths = export_perception_result(
            args.perception_output_dir,
            perception_result,
        )
        perception_html = write_perception_html(
            sensor_result.observation,
            perception_result,
            args.perception_output_dir / "geometry.html",
        ).resolve()
        perception_metrics = perception_result.metrics
        print(
            "RGB-D 幾何："
            f"物件={perception_metrics['detected_object_count']}｜"
            f"抓取候選={perception_metrics['grasp_candidate_count']}｜"
            f"幾何可行={perception_metrics['feasible_grasp_candidate_count']}｜"
            f"桌面 RMS={perception_metrics['table_plane_rms_error_m'] * 1000.0:.2f} mm"
        )
        print(f"感知結果：{perception_paths['geometry'].resolve()}")
        print(f"感知視覺化：{perception_html}")

    replay_result = None
    if (
        physics_sweep is not None
        and perception_result is not None
        and not args.no_build_replay
    ):
        integration_spec = load_integration_spec(args.integration_config)
        replay_result = build_fail_closed_replay(
            physics_sweep.baseline,
            perception_result,
            spec.robot,
            integration_spec,
        )
        description_paths = export_robot_description(
            args.integration_output_dir,
            spec.robot,
        )
        replay_paths = export_replay_result(
            args.integration_output_dir,
            replay_result,
        )
        status = "AUTHORIZED" if replay_result.execution_authorized else "ABORTED"
        print(
            "安全重播："
            f"{status}｜失敗分類={len(replay_result.failure_codes)}｜"
            f"控制幀={replay_result.metrics['command_frame_count']}"
        )
        print(f"URDF/SRDF：{description_paths['urdf'].resolve()}")
        print(f"控制重播：{replay_paths['replay'].resolve()}")

    report_path: Path | None = None
    if sensor_result is not None:
        report_path = write_simulation_report(
            scene_data,
            sensor_result,
            args.report_output,
            trajectory=motion_trajectory,
            physics_sweep=physics_sweep,
            perception=perception_result,
            replay=replay_result,
        ).resolve()
        print(f"單頁驗證報告：{report_path}")

    if args.open:
        browser_target = report_path or motion_path or html_path
        webbrowser.open(browser_target.as_uri())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
