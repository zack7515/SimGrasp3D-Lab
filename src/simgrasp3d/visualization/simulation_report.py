"""將世界場景與 RGB-D 感測結果組成單頁雙視窗報告。"""

from __future__ import annotations

from html import escape
from pathlib import Path

from plotly.io import to_html
from plotly.offline import get_plotlyjs

from simgrasp3d.models.motion import TrajectoryData
from simgrasp3d.scene.builder import SceneData
from simgrasp3d.sensors.rgbd import RGBDSimulationResult
from simgrasp3d.visualization.motion_viewer import build_motion_figure
from simgrasp3d.visualization.plotly_viewer import build_figure
from simgrasp3d.visualization.rgbd_viewer import build_rgbd_comparison_figure


_REPORT_CSS = """
:root {
  --graphite: #101820;
  --steel: #263746;
  --slate: #536777;
  --ice: #edf4f6;
  --paper: #f8fbfc;
  --cyan: #18ad9c;
  --cyan-soft: #d9f2ed;
  --amber: #e89a36;
  --amber-soft: #fff0d9;
  --danger: #d85c4a;
  --line: #cbd8dd;
}

* { box-sizing: border-box; }

body {
  margin: 0;
  color: var(--graphite);
  background:
    linear-gradient(rgba(38, 55, 70, 0.045) 1px, transparent 1px),
    linear-gradient(90deg, rgba(38, 55, 70, 0.045) 1px, transparent 1px),
    var(--ice);
  background-size: 24px 24px;
  font-family: "Noto Sans TC", "PingFang TC", "Microsoft JhengHei", system-ui, sans-serif;
}

.instrument-bar {
  color: #f5fbfc;
  background: var(--graphite);
  border-bottom: 4px solid var(--cyan);
  padding: 22px clamp(20px, 3vw, 46px) 20px;
}

.instrument-grid {
  display: grid;
  grid-template-columns: minmax(0, 1fr) auto;
  gap: 24px;
  align-items: end;
}

.eyebrow,
.pane-code,
.meta-label,
.metric-label,
.condition-label,
.rail-label {
  font-family: ui-monospace, "SFMono-Regular", Consolas, monospace;
  letter-spacing: 0.11em;
  text-transform: uppercase;
}

.eyebrow {
  margin: 0 0 8px;
  color: #8ee2d8;
  font-size: 12px;
}

h1 {
  margin: 0;
  max-width: 900px;
  font-family: "DIN Condensed", "Roboto Condensed", "Arial Narrow", sans-serif;
  font-size: clamp(30px, 4vw, 58px);
  font-stretch: condensed;
  font-weight: 700;
  letter-spacing: -0.025em;
  line-height: 0.98;
}

.run-state {
  display: grid;
  grid-template-columns: repeat(2, auto);
  gap: 8px;
}

.state-cell {
  min-width: 108px;
  padding: 9px 12px;
  border: 1px solid #3c5364;
  background: #17242e;
}

.state-cell span { display: block; }

.state-cell .meta-label {
  color: #8ea5b5;
  font-size: 9px;
}

.state-cell .meta-value {
  margin-top: 4px;
  color: #ffffff;
  font-family: ui-monospace, "SFMono-Regular", Consolas, monospace;
  font-size: 13px;
}

main {
  width: min(1920px, 100%);
  margin: 0 auto;
  padding: 18px clamp(12px, 2vw, 30px) 36px;
}

.context-strip {
  display: grid;
  grid-template-columns: 1.2fr 1fr;
  gap: 12px;
  margin-bottom: 14px;
}

.context-card {
  min-width: 0;
  padding: 13px 16px;
  border: 1px solid var(--line);
  background: rgba(248, 251, 252, 0.92);
  box-shadow: 0 8px 24px rgba(16, 24, 32, 0.06);
}

.context-card h2 {
  margin: 0 0 8px;
  font-size: 13px;
  font-weight: 700;
}

.condition-list {
  display: flex;
  flex-wrap: wrap;
  gap: 7px;
}

.condition {
  display: inline-flex;
  gap: 7px;
  align-items: baseline;
  padding: 6px 8px;
  border-left: 3px solid var(--slate);
  background: #edf2f4;
  font-size: 12px;
}

.condition.noise {
  border-left-color: var(--amber);
  background: var(--amber-soft);
}

.condition-label {
  color: var(--slate);
  font-size: 9px;
}

.workbench {
  position: relative;
  display: grid;
  grid-template-columns: minmax(0, 1.04fr) minmax(0, 0.96fr);
  gap: 14px;
}

.calibration-rail {
  position: absolute;
  z-index: 4;
  top: 54px;
  bottom: 18px;
  left: 52%;
  width: 1px;
  background: repeating-linear-gradient(
    to bottom,
    var(--cyan) 0,
    var(--cyan) 9px,
    transparent 9px,
    transparent 15px
  );
  pointer-events: none;
}

.rail-label {
  position: absolute;
  top: 16px;
  left: 50%;
  transform: translateX(-50%);
  padding: 5px 8px;
  color: var(--graphite);
  background: var(--cyan-soft);
  border: 1px solid #9dd8cf;
  font-size: 9px;
  white-space: nowrap;
}

.pane {
  min-width: 0;
  overflow: hidden;
  border: 1px solid #b9c9cf;
  background: var(--paper);
  box-shadow: 0 14px 35px rgba(16, 24, 32, 0.10);
}

.pane-header {
  display: flex;
  justify-content: space-between;
  gap: 18px;
  align-items: center;
  min-height: 54px;
  padding: 10px 14px;
  border-bottom: 1px solid var(--line);
  background: #ffffff;
}

.pane-title {
  display: flex;
  gap: 10px;
  align-items: center;
}

.pane-code {
  display: inline-grid;
  width: 30px;
  height: 30px;
  place-items: center;
  color: #ffffff;
  background: var(--steel);
  font-size: 11px;
}

.pane.sensor .pane-code { background: var(--amber); }

.pane-header h2 {
  margin: 0;
  font-size: 15px;
}

.pane-header p {
  margin: 2px 0 0;
  color: var(--slate);
  font-size: 11px;
}

.pane-badge {
  flex: 0 0 auto;
  padding: 5px 8px;
  color: #116e63;
  background: var(--cyan-soft);
  font-family: ui-monospace, "SFMono-Regular", Consolas, monospace;
  font-size: 10px;
}

.pane.sensor .pane-badge {
  color: #895817;
  background: var(--amber-soft);
}

.plot-shell {
  width: 100%;
  min-height: 650px;
  background: #f7fafb;
}

.plot-shell > div { width: 100% !important; }

.results {
  margin-top: 14px;
  border: 1px solid var(--line);
  background: rgba(248, 251, 252, 0.96);
  box-shadow: 0 10px 30px rgba(16, 24, 32, 0.07);
}

.results-header {
  display: flex;
  justify-content: space-between;
  gap: 16px;
  align-items: baseline;
  padding: 14px 16px 10px;
  border-bottom: 1px solid var(--line);
}

.results-header h2 {
  margin: 0;
  font-size: 16px;
}

.results-header p {
  margin: 0;
  color: var(--slate);
  font-size: 11px;
}

.metric-grid {
  display: grid;
  grid-template-columns: repeat(6, minmax(110px, 1fr));
  border-bottom: 1px solid var(--line);
}

.metric {
  min-width: 0;
  padding: 14px 16px;
  border-right: 1px solid var(--line);
}

.metric:last-child { border-right: 0; }

.metric-label {
  display: block;
  min-height: 24px;
  color: var(--slate);
  font-size: 9px;
  line-height: 1.35;
}

.metric-value {
  display: block;
  margin-top: 7px;
  font-family: "DIN Condensed", "Roboto Condensed", "Arial Narrow", sans-serif;
  font-size: clamp(24px, 2vw, 34px);
  font-weight: 700;
  line-height: 1;
}

.metric.accent .metric-value { color: #0f8175; }
.metric.warning .metric-value { color: #b36d17; }

.metric-table-wrap {
  overflow-x: auto;
  padding: 10px 16px 14px;
}

.metric-table {
  width: 100%;
  border-collapse: collapse;
  font-size: 12px;
}

.metric-table th,
.metric-table td {
  padding: 8px 10px;
  border-bottom: 1px solid #dce5e8;
  text-align: left;
}

.metric-table th {
  width: 34%;
  color: var(--slate);
  font-family: ui-monospace, "SFMono-Regular", Consolas, monospace;
  font-size: 10px;
  font-weight: 500;
}

.metric-table td {
  font-family: ui-monospace, "SFMono-Regular", Consolas, monospace;
  font-variant-numeric: tabular-nums;
}

.report-note {
  margin: 12px 2px 0;
  padding-left: 10px;
  color: #526572;
  border-left: 3px solid var(--amber);
  font-size: 11px;
  line-height: 1.6;
}

.motion-section {
  margin-top: 14px;
}

.motion-section .pane-code { background: var(--cyan); }

.phase-rail {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(105px, 1fr));
  border-bottom: 1px solid var(--line);
  background: #edf3f4;
}

.phase-step {
  position: relative;
  min-width: 0;
  padding: 10px 10px 10px 34px;
  border-right: 1px solid #d4dfe2;
  color: var(--steel);
  font-size: 10px;
  line-height: 1.35;
}

.phase-step:last-child { border-right: 0; }

.phase-index {
  position: absolute;
  top: 9px;
  left: 9px;
  display: grid;
  width: 18px;
  height: 18px;
  place-items: center;
  color: #ffffff;
  background: var(--steel);
  border-radius: 50%;
  font-family: ui-monospace, "SFMono-Regular", Consolas, monospace;
  font-size: 9px;
}

.motion-plot {
  min-height: 720px;
  background: #f7fafb;
}

.motion-plot > div { width: 100% !important; }

.motion-note {
  padding: 11px 16px;
  color: var(--slate);
  background: #f1f5f6;
  border-top: 1px solid var(--line);
  font-size: 11px;
  line-height: 1.6;
}

@media (max-width: 1180px) {
  .workbench,
  .context-strip { grid-template-columns: 1fr; }
  .calibration-rail { display: none; }
  .metric-grid { grid-template-columns: repeat(3, 1fr); }
  .metric:nth-child(3) { border-right: 0; }
  .metric:nth-child(-n + 3) { border-bottom: 1px solid var(--line); }
}

@media (max-width: 680px) {
  .instrument-grid { grid-template-columns: 1fr; }
  .run-state { grid-template-columns: repeat(2, minmax(0, 1fr)); }
  .state-cell { min-width: 0; }
  .metric-grid { grid-template-columns: repeat(2, 1fr); }
  .metric:nth-child(3) { border-right: 1px solid var(--line); }
  .metric:nth-child(even) { border-right: 0; }
  .metric:nth-child(-n + 4) { border-bottom: 1px solid var(--line); }
  .pane-header { align-items: flex-start; }
  .pane-badge { display: none; }
  .plot-shell { min-height: 560px; }
  .motion-plot { min-height: 620px; }
}

@media (prefers-reduced-motion: reduce) {
  *, *::before, *::after { scroll-behavior: auto !important; }
}
"""


_METRIC_LABELS = {
    "total_pixels": "影像總像素",
    "ground_truth_valid_pixels": "Ground truth 有效像素",
    "observation_valid_pixels": "Observation 有效像素",
    "common_valid_pixels": "共同有效像素",
    "ground_truth_fill_ratio": "Ground truth 填充率",
    "observation_fill_ratio": "Observation 填充率",
    "common_retention_ratio": "共同像素保留率",
    "depth_mae_m": "深度 MAE",
    "depth_rmse_m": "深度 RMSE",
    "depth_bias_m": "深度 Bias",
    "depth_p95_abs_error_m": "絕對誤差 P95",
    "injected_depth_dropouts": "注入深度孔洞",
    "extrinsic_translation_error_norm_m": "外參平移誤差範數",
    "extrinsic_rotation_error_norm_deg": "外參旋轉誤差範數",
}


def _format_metric(key: str, value: float | int) -> str:
    """依指標物理意義轉成適合閱讀的單位。"""

    if key.endswith("_ratio"):
        return f"{float(value) * 100.0:.2f}%"
    if key.startswith("depth_") and key.endswith("_m"):
        return f"{float(value) * 1000.0:.2f} mm"
    if key == "extrinsic_translation_error_norm_m":
        return f"{float(value) * 1000.0:.3f} mm"
    if key == "extrinsic_rotation_error_norm_deg":
        return f"{float(value):.3f}°"
    if isinstance(value, int):
        return f"{value:,}"
    return f"{float(value):.6f}"


def _metric_table(result: RGBDSimulationResult) -> str:
    """建立包含全部量化結果的表格。"""

    rows: list[str] = []
    for key, value in result.metrics.items():
        label = _METRIC_LABELS.get(key, key)
        rows.append(
            "<tr>"
            f"<th scope=\"row\">{escape(label)}</th>"
            f"<td>{escape(_format_metric(key, value))}</td>"
            f"<td>{escape(key)}</td>"
            "</tr>"
        )
    return "".join(rows)


def _condition(label: str, value: str, noise: bool = False) -> str:
    """建立一個測試條件標籤。"""

    class_name = "condition noise" if noise else "condition"
    return (
        f'<span class="{class_name}">'
        f'<span class="condition-label">{escape(label)}</span>'
        f"<strong>{escape(value)}</strong>"
        "</span>"
    )


def _motion_metric_table(trajectory: TrajectoryData) -> str:
    """建立運動學、碰撞與軟管約束的完整指標表。"""

    labels = {
        "frame_count": "動畫幀數",
        "duration_s": "動作總時間",
        "minimum_clearance_m": "機器人最小距離（相容欄位）",
        "minimum_robot_clearance_m": "機器人最小環境距離",
        "minimum_link_clearance_m": "連桿最小環境距離",
        "minimum_gripper_clearance_m": "夾爪最小環境距離",
        "minimum_hose_clearance_m": "軟管最小管路距離",
        "collision_frame_count": "機器人碰撞幀數",
        "unsafe_clearance_frame_count": "機器人安全警示幀數",
        "hose_contact_frame_count": "軟管接觸幀數",
        "hose_penetration_frame_count": "軟管穿透幀數",
        "maximum_ik_error_m": "最大 IK 位置誤差",
        "maximum_ik_orientation_error_deg": "最大 IK 姿態誤差",
        "failed_ik_frame_count": "IK 失敗幀數",
        "maximum_hose_length_error_ratio": "最大軟管長度誤差",
        "attached_frame_count": "夾持狀態幀數",
        "planned_keyframe_count": "規劃後關鍵幀數",
        "inserted_waypoint_count": "自動插入 waypoint 數",
        "unresolved_path_segment_count": "未解決路徑線段數",
    }
    rows = []
    for key, value in trajectory.metrics.items():
        if key.endswith("_m"):
            display = f"{float(value) * 1000.0:.3f} mm"
        elif key.endswith("_deg"):
            display = f"{float(value):.3f}°"
        elif key.endswith("_ratio"):
            display = f"{float(value) * 100.0:.3f}%"
        elif key == "duration_s":
            display = f"{float(value):.2f} s"
        else:
            display = f"{int(value):,}"
        rows.append(
            "<tr>"
            f'<th scope="row">{escape(labels.get(key, key))}</th>'
            f"<td>{escape(display)}</td>"
            f"<td>{escape(key)}</td>"
            "</tr>"
        )
    return "".join(rows)


def _motion_section(
    scene_data: SceneData,
    trajectory: TrajectoryData | None,
) -> str:
    """建立可選的 C 區連續動作動畫與量化結果。"""

    if trajectory is None:
        return ""
    figure = build_motion_figure(
        trajectory,
        scene_data.spec.robot,
        scene_data.spec.table,
    )
    motion_html = to_html(
        figure,
        include_plotlyjs=False,
        full_html=False,
        div_id="motion-view",
        config={"displaylogo": False, "responsive": True, "scrollZoom": True},
    )
    phases: list[str] = []
    for keyframe in trajectory.planned_keyframes:
        if not phases or phases[-1] != keyframe.phase:
            phases.append(keyframe.phase)
    phase_rail = "".join(
        f'<div class="phase-step"><span class="phase-index">{index:02d}</span>{escape(phase)}</div>'
        for index, phase in enumerate(phases, start=1)
    )
    metrics = trajectory.metrics
    motion_cards = "".join(
        (
            f'<div class="metric {class_name}">'
            f'<span class="metric-label">{escape(label)}</span>'
            f'<span class="metric-value">{escape(value)}</span>'
            "</div>"
        )
        for label, value, class_name in (
            ("DURATION", f"{float(metrics['duration_s']):.1f} s", "accent"),
            ("AUTO WAYPOINT", f"{int(metrics['inserted_waypoint_count'])}", "warning"),
            ("IK POSITION", f"{float(metrics['maximum_ik_error_m']) * 1000.0:.2f} mm", ""),
            ("IK ORIENTATION", f"{float(metrics['maximum_ik_orientation_error_deg']):.2f}°", ""),
            ("ROBOT CLEARANCE", f"{float(metrics['minimum_robot_clearance_m']) * 1000.0:.1f} mm", "accent"),
            ("ROBOT WARN / HIT", f"{int(metrics['unsafe_clearance_frame_count'])} / {int(metrics['collision_frame_count'])}", "accent"),
        )
    )
    return f"""
    <section class="pane motion-section" aria-label="軟管夾取連續動作動畫">
      <header class="pane-header">
        <div class="pane-title"><span class="pane-code">C</span><div><h2>軟管抽取與搬運時間序列</h2><p>六自由度 IK、機器人尺寸碰撞、軟管接觸與自動 waypoint</p></div></div>
        <span class="pane-badge">KINEMATIC LEARNING</span>
      </header>
      <div class="phase-rail" aria-label="動作階段">{phase_rail}</div>
      <div class="motion-plot">{motion_html}</div>
      <div class="metric-grid">{motion_cards}</div>
      <div class="metric-table-wrap">
        <table class="metric-table">
          <thead><tr><th>運動指標</th><th>顯示值</th><th>資料欄位</th></tr></thead>
          <tbody>{_motion_metric_table(trajectory)}</tbody>
        </table>
      </div>
      <div class="motion-note">TCP 橘／紅色代表機器人低於 {trajectory.spec.safe_clearance_m * 1000.0:.1f} mm 或穿透；軟管節點橘色則表示與管路進入 1 mm 接觸帶，兩者分開統計。規劃器插入的 waypoint 以橘色路徑節點顯示。此階段仍不計算材料剛性、摩擦、接觸力或慣性。</div>
    </section>
    """


def write_simulation_report(
    scene_data: SceneData,
    result: RGBDSimulationResult,
    output_path: str | Path,
    trajectory: TrajectoryData | None = None,
) -> Path:
    """輸出世界、感測比較與可選連續動作的單頁報告。"""

    destination = Path(output_path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    scene_figure = build_figure(scene_data)
    scene_figure.update_layout(
        title=None,
        height=650,
        margin={"l": 0, "r": 0, "t": 12, "b": 0},
        paper_bgcolor="#f7fafb",
    )
    sensor_figure = build_rgbd_comparison_figure(result)
    sensor_figure.update_layout(
        title=None,
        height=650,
        margin={"l": 28, "r": 70, "t": 54, "b": 28},
        paper_bgcolor="#f7fafb",
    )
    scene_html = to_html(
        scene_figure,
        include_plotlyjs=False,
        full_html=False,
        div_id="world-view",
        config={"displaylogo": False, "responsive": True, "scrollZoom": True},
    )
    sensor_html = to_html(
        sensor_figure,
        include_plotlyjs=False,
        full_html=False,
        div_id="sensor-view",
        config={"displaylogo": False, "responsive": True},
    )

    spec = scene_data.spec
    camera = spec.camera
    noise = camera.noise
    total_points = sum(cloud.points.shape[0] for cloud in scene_data.point_clouds)
    object_names = "、".join(item.name for item in spec.objects)
    conditions = "".join(
        (
            _condition("模式", "固定 eye-to-hand / 靜態桌面"),
            _condition("相機", f"{camera.width}×{camera.height} / FOV {camera.vertical_fov_deg:g}°"),
            _condition("深度範圍", f"{camera.near:g}–{camera.far:g} m"),
            _condition("量化", f"{noise.depth_quantization_m * 1000.0:g} mm", True),
            _condition("孔洞", f"{noise.dropout_probability * 100.0:g}%", True),
            _condition(
                "軸向雜訊",
                f"{noise.axial_noise_std_base_m * 1000.0:g} mm + z²×{noise.axial_noise_std_per_m2:g}",
                True,
            ),
            _condition(
                "外參 σ",
                f"{noise.extrinsic_translation_std_m * 1000.0:g} mm / {noise.extrinsic_rotation_std_deg:g}°",
                True,
            ),
        )
    )
    metrics = result.metrics
    key_metrics = (
        ("GROUND TRUTH", _format_metric("ground_truth_valid_pixels", metrics["ground_truth_valid_pixels"]), "accent"),
        ("OBSERVATION", _format_metric("observation_valid_pixels", metrics["observation_valid_pixels"]), "warning"),
        ("COMMON RETENTION", _format_metric("common_retention_ratio", metrics["common_retention_ratio"]), ""),
        ("DEPTH MAE", _format_metric("depth_mae_m", metrics["depth_mae_m"]), "warning"),
        ("DEPTH RMSE", _format_metric("depth_rmse_m", metrics["depth_rmse_m"]), "warning"),
        ("ERROR P95", _format_metric("depth_p95_abs_error_m", metrics["depth_p95_abs_error_m"]), "warning"),
    )
    metric_cards = "".join(
        (
            f'<div class="metric {class_name}">'
            f'<span class="metric-label">{escape(label)}</span>'
            f'<span class="metric-value">{escape(value)}</span>'
            "</div>"
        )
        for label, value, class_name in key_metrics
    )
    motion_section = _motion_section(scene_data, trajectory)
    motion_resize = (
        'Plotly.Plots.resize(document.getElementById("motion-view"));'
        if trajectory is not None
        else ""
    )
    page_title = f"SimGrasp3D｜{spec.name}｜模擬驗證報告"
    document = f"""<!doctype html>
<html lang="zh-Hant">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>{escape(page_title)}</title>
  <style>{_REPORT_CSS}</style>
  <script>{get_plotlyjs()}</script>
</head>
<body>
  <header class="instrument-bar">
    <div class="instrument-grid">
      <div>
        <p class="eyebrow">SimGrasp3D / simulation validation</p>
        <h1>世界座標 vs. RGB-D 觀測</h1>
      </div>
      <div class="run-state" aria-label="執行識別">
        <div class="state-cell"><span class="meta-label">Scene</span><span class="meta-value">{escape(spec.name)}</span></div>
        <div class="state-cell"><span class="meta-label">Seed</span><span class="meta-value">{spec.seed}</span></div>
        <div class="state-cell"><span class="meta-label">Entities</span><span class="meta-value">{len(scene_data.point_clouds)}</span></div>
        <div class="state-cell"><span class="meta-label">Points</span><span class="meta-value">{total_points:,}</span></div>
      </div>
    </div>
  </header>
  <main>
    <section class="context-strip" aria-label="測試情境與條件">
      <div class="context-card">
        <h2>測試情境</h2>
        <div class="condition-list">
          {_condition("場景", "靜態桌面 + 六軸手臂")}
          {_condition("物件", object_names)}
          {_condition("Ground truth", "名義外參 / 無深度誤差")}
          {_condition("Observation", "實際姿態擾動 / 名義外參回投影", True)}
        </div>
      </div>
      <div class="context-card">
        <h2>感測條件</h2>
        <div class="condition-list">{conditions}</div>
      </div>
    </section>

    <section class="workbench" aria-label="原始場景與感測結果雙畫面比較">
      <div class="calibration-rail"><span class="rail-label">WORLD → SENSOR</span></div>
      <article class="pane world">
        <header class="pane-header">
          <div class="pane-title"><span class="pane-code">A</span><div><h2>原始模擬世界</h2><p>完整幾何、機械手、TCP、座標系與相機視錐</p></div></div>
          <span class="pane-badge">GROUND TRUTH WORLD</span>
        </header>
        <div class="plot-shell">{scene_html}</div>
      </article>
      <article class="pane sensor">
        <header class="pane-header">
          <div class="pane-title"><span class="pane-code">B</span><div><h2>相機測試結果</h2><p>理想深度、含誤差觀測、絕對誤差與 RGB</p></div></div>
          <span class="pane-badge">SENSOR OBSERVATION</span>
        </header>
        <div class="plot-shell">{sensor_html}</div>
      </article>
    </section>

    <section class="results" aria-label="全部量化測試結果">
      <header class="results-header"><h2>量化結果</h2><p>深度誤差只在 ground truth 與 observation 的共同有效像素計算</p></header>
      <div class="metric-grid">{metric_cards}</div>
      <div class="metric-table-wrap">
        <table class="metric-table">
          <thead><tr><th>指標</th><th>顯示值</th><th>資料欄位</th></tr></thead>
          <tbody>{_metric_table(result)}</tbody>
        </table>
      </div>
    </section>
    {motion_section}
    <p class="report-note">此頁是模擬結果，不是實機驗證。較大的深度邊界誤差會同時包含外參偏移後不同表面落入同一像素的影響；目前的影像填充率也受離散表面點數影響。</p>
  </main>
  <script>
    window.addEventListener("load", function () {{
      window.setTimeout(function () {{
        Plotly.Plots.resize(document.getElementById("world-view"));
        Plotly.Plots.resize(document.getElementById("sensor-view"));
        {motion_resize}
      }}, 80);
    }});
  </script>
</body>
</html>
"""
    destination.write_text(document, encoding="utf-8")
    return destination
