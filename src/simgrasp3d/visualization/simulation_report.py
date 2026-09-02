"""將世界場景與 RGB-D 感測結果組成單頁雙視窗報告。"""

from __future__ import annotations

from html import escape
from pathlib import Path

from plotly.io import to_html
from plotly.offline import get_plotlyjs

from simgrasp3d.models.integration import ReplayResult
from simgrasp3d.models.motion import TrajectoryData
from simgrasp3d.models.perception import PerceptionResult
from simgrasp3d.models.physics import PhysicsSweepData
from simgrasp3d.scene.builder import SceneData
from simgrasp3d.sensors.rgbd import RGBDSimulationResult
from simgrasp3d.visualization.motion_viewer import build_motion_figure
from simgrasp3d.visualization.perception_viewer import build_perception_figure
from simgrasp3d.visualization.physics_viewer import build_physics_comparison_figure
from simgrasp3d.visualization.plotly_viewer import build_figure
from simgrasp3d.visualization.rgbd_viewer import build_rgbd_comparison_figure
from simgrasp3d.visualization.theme import CERAMIC


_REPORT_CSS = """
:root {
  --graphite: #11181d;
  --steel: #2a3942;
  --slate: #60727a;
  --ice: #e9efed;
  --paper: #f4f7f6;
  --cyan: #0aa58f;
  --cyan-dark: #087568;
  --cyan-soft: #d8eee9;
  --amber: #e7a33b;
  --amber-soft: #faecd4;
  --danger: #c54437;
  --line: #d6e0de;
  --display: "Bahnschrift SemiCondensed", "DIN Alternate", "Arial Narrow", sans-serif;
  --body: Aptos, "Noto Sans TC", "PingFang TC", "Microsoft JhengHei", sans-serif;
  --mono: "JetBrains Mono", "IBM Plex Mono", "SFMono-Regular", Consolas, monospace;
}

* { box-sizing: border-box; }

html { scroll-behavior: smooth; }

body {
  margin: 0;
  color: var(--graphite);
  background:
    linear-gradient(rgba(42, 57, 66, 0.035) 1px, transparent 1px),
    linear-gradient(90deg, rgba(42, 57, 66, 0.035) 1px, transparent 1px),
    radial-gradient(circle at 50% -18%, rgba(10, 165, 143, 0.12), transparent 34%),
    var(--ice);
  background-size: 40px 40px, 40px 40px, auto, auto;
  font-family: var(--body);
  line-height: 1.5;
}

.skip-link {
  position: fixed;
  z-index: 100;
  top: 8px;
  left: 8px;
  padding: 9px 12px;
  color: var(--graphite);
  background: #ffd966;
  border-bottom: 4px solid var(--graphite);
  transform: translateY(-160%);
}

.skip-link:focus { transform: translateY(0); }

a:focus-visible,
summary:focus-visible {
  outline: 3px solid #ffd966;
  outline-offset: 3px;
}

.instrument-bar {
  color: #f5fbfc;
  background: var(--graphite);
  border-bottom: 1px solid #354852;
  padding: 24px clamp(20px, 3vw, 46px) 18px;
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
  font-family: var(--mono);
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
  font-family: var(--display);
  font-size: clamp(30px, 4vw, 58px);
  font-stretch: condensed;
  font-weight: 700;
  letter-spacing: -0.018em;
  line-height: 0.96;
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
  font-family: var(--mono);
  font-size: 13px;
}

.signal-path {
  position: relative;
  display: grid;
  grid-template-columns: repeat(4, minmax(0, 1fr));
  margin-top: 20px;
  padding-top: 12px;
  border-top: 1px solid #354852;
  overflow: hidden;
}

.signal-path::before {
  content: "";
  position: absolute;
  top: 18px;
  right: 7%;
  left: 7%;
  height: 1px;
  background: #49606c;
}

.signal-pulse {
  position: absolute;
  z-index: 2;
  top: 14px;
  left: 7%;
  width: 9px;
  height: 9px;
  border-radius: 50%;
  background: #b7fff3;
  box-shadow: 0 0 0 4px rgba(10, 165, 143, 0.18), 0 0 18px #0aa58f;
  animation: calibrate-signal 5.4s cubic-bezier(0.4, 0, 0.2, 1) infinite;
}

.signal-node {
  position: relative;
  z-index: 3;
  padding: 0 12px;
  text-align: center;
}

.signal-node::before {
  content: "";
  display: block;
  width: 9px;
  height: 9px;
  margin: 2px auto 8px;
  border: 2px solid #8ca0a9;
  border-radius: 50%;
  background: var(--graphite);
}

.signal-node.is-live::before { border-color: #7ee5d6; background: var(--cyan); }
.signal-node.is-fault::before { border-color: #ff9c90; background: var(--danger); }

.signal-node span,
.signal-node strong { display: block; }

.signal-node span {
  color: #879ba5;
  font-family: var(--mono);
  font-size: 8px;
  letter-spacing: 0.12em;
}

.signal-node strong {
  margin-top: 2px;
  color: #edf7f4;
  font-family: var(--mono);
  font-size: 10px;
  font-weight: 500;
}

@keyframes calibrate-signal {
  0%, 8% { left: 7%; opacity: 0; }
  14% { opacity: 1; }
  86% { opacity: 1; }
  94%, 100% { left: calc(93% - 9px); opacity: 0; }
}

.chapter-nav {
  position: sticky;
  z-index: 20;
  top: 0;
  display: grid;
  grid-template-columns: repeat(var(--stage-count), minmax(90px, 1fr));
  width: min(1920px, 100%);
  margin: 0 auto;
  padding: 0 clamp(12px, 2vw, 30px);
  background: rgba(233, 239, 237, 0.94);
  border-bottom: 1px solid #bdcac7;
  backdrop-filter: blur(14px);
}

.chapter-nav a {
  position: relative;
  display: flex;
  gap: 8px;
  align-items: center;
  justify-content: center;
  min-height: 46px;
  color: var(--slate);
  border-right: 1px solid var(--line);
  font-size: 11px;
  text-decoration: none;
}

.chapter-nav a:first-child { border-left: 1px solid var(--line); }

.chapter-nav a::after {
  content: "";
  position: absolute;
  right: 14px;
  bottom: -1px;
  left: 14px;
  height: 3px;
  background: transparent;
}

.chapter-nav a:hover,
.chapter-nav a.is-active { color: var(--graphite); }
.chapter-nav a.is-active::after { background: var(--cyan); }

.chapter-nav span {
  display: grid;
  width: 22px;
  height: 22px;
  place-items: center;
  color: #ffffff;
  background: var(--steel);
  font-family: var(--mono);
  font-size: 9px;
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
  background: rgba(244, 247, 246, 0.94);
  box-shadow: 0 6px 18px rgba(17, 24, 29, 0.05);
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
  border: 1px solid #bac8c5;
  background: var(--paper);
  box-shadow: 0 12px 30px rgba(17, 24, 29, 0.08);
  scroll-margin-top: 62px;
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
  font-family: var(--display);
  font-size: 15px;
  letter-spacing: 0.015em;
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
  font-family: var(--mono);
  font-size: 10px;
}

.pane.sensor .pane-badge {
  color: #895817;
  background: var(--amber-soft);
}

.plot-shell {
  width: 100%;
  min-height: 650px;
  background: var(--paper);
}

.plot-shell > div { width: 100% !important; }

.results {
  margin-top: 14px;
  border: 1px solid var(--line);
  background: rgba(244, 247, 246, 0.97);
  box-shadow: 0 8px 24px rgba(17, 24, 29, 0.06);
  scroll-margin-top: 62px;
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
  font-family: var(--display);
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
  font-family: var(--display);
  font-size: clamp(24px, 2vw, 34px);
  font-weight: 700;
  line-height: 1;
}

.metric.accent .metric-value { color: var(--cyan-dark); }
.metric.warning .metric-value { color: #9b6517; }

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
  font-family: var(--mono);
  font-size: 10px;
  font-weight: 500;
}

.metric-table td {
  font-family: var(--mono);
  font-variant-numeric: tabular-nums;
}

.metric-table tbody tr:hover { background: rgba(10, 165, 143, 0.055); }

.report-note {
  margin: 12px 2px 0;
  padding-left: 10px;
  color: #526572;
  border-left: 3px solid var(--amber);
  font-size: 11px;
  line-height: 1.6;
}

.motion-section,
.analysis-section {
  margin-top: 14px;
}

.motion-section .pane-code,
.analysis-section .pane-code { background: var(--cyan); }

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
  font-family: var(--mono);
  font-size: 9px;
}

.motion-plot {
  min-height: 720px;
  background: var(--paper);
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

.metric-disclosure {
  border-top: 1px solid var(--line);
  background: #eef3f1;
}

.metric-disclosure summary {
  display: flex;
  justify-content: space-between;
  gap: 16px;
  padding: 11px 16px;
  color: var(--steel);
  cursor: pointer;
  font-family: var(--mono);
  font-size: 10px;
  letter-spacing: 0.05em;
  list-style: none;
}

.metric-disclosure summary::-webkit-details-marker { display: none; }
.metric-disclosure summary::after { content: "+ 展開"; color: var(--cyan-dark); }
.metric-disclosure[open] summary::after { content: "− 收合"; }
.metric-disclosure[open] summary { border-bottom: 1px solid var(--line); }

.gate-banner {
  display: flex;
  gap: 12px;
  align-items: center;
  justify-content: space-between;
  padding: 12px 16px;
  color: #ffffff;
  background: var(--cyan-dark);
  font-family: var(--mono);
  font-size: 11px;
}

.gate-banner.is-fault { background: var(--danger); }
.gate-banner strong { font-size: 13px; letter-spacing: 0.08em; }

body.motion-ready .reveal {
  opacity: 0;
  transform: translateY(18px);
}

body.motion-ready .reveal.is-visible {
  opacity: 1;
  transform: translateY(0);
  transition: opacity 420ms ease-out, transform 520ms cubic-bezier(0.2, 0.7, 0.2, 1);
}

@media (max-width: 1180px) {
  .workbench,
  .context-strip { grid-template-columns: 1fr; }
  .calibration-rail { display: none; }
  .metric-grid { grid-template-columns: repeat(3, 1fr); }
  .metric:nth-child(3) { border-right: 0; }
  .metric:nth-child(-n + 3) { border-bottom: 1px solid var(--line); }
  .chapter-nav { overflow-x: auto; justify-content: start; }
  .chapter-nav a { min-width: 112px; }
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
  .signal-node { padding: 0 3px; }
  .signal-node span { font-size: 7px; }
  .signal-node strong { font-size: 8px; }
  .chapter-nav { padding: 0 8px; }
}

@media (prefers-reduced-motion: reduce) {
  *, *::before, *::after {
    scroll-behavior: auto !important;
    animation-duration: 0.01ms !important;
    animation-iteration-count: 1 !important;
    transition-duration: 0.01ms !important;
  }
  body.motion-ready .reveal { opacity: 1; transform: none; }
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
        "physics_step_count": "MuJoCo 步進數",
        "maximum_physics_contact_count": "單幀環境接觸數",
        "maximum_contact_force_n": "抽樣最大環境接觸力",
        "minimum_contact_distance_m": "最深環境接觸距離",
        "maximum_physics_self_contact_count": "單幀自接觸數",
        "maximum_self_contact_force_n": "抽樣最大自接觸力",
        "minimum_self_contact_distance_m": "最深自接觸距離",
        "maximum_grasp_constraint_error_m": "最大抓持約束誤差",
        "maximum_hose_speed_m_s": "輸出幀最大節點速度",
        "maximum_total_energy_j": "最大總能量",
        "physics_nonfinite_frame_count": "非有限值幀數",
    }
    rows = []
    for key, value in trajectory.metrics.items():
        if key.endswith("_m"):
            display = f"{float(value) * 1000.0:.3f} mm"
        elif key.endswith("_deg"):
            display = f"{float(value):.3f}°"
        elif key.endswith("_ratio"):
            display = f"{float(value) * 100.0:.3f}%"
        elif key.endswith("_n"):
            display = f"{float(value):.3f} N"
        elif key.endswith("_j"):
            display = f"{float(value):.5f} J"
        elif key.endswith("_m_s"):
            display = f"{float(value):.3f} m/s"
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
    engine_label = trajectory.physics_engine or "幾何運動學"
    section_badge = "PHYSICS REPLAY" if trajectory.physics_engine else "KINEMATIC LEARNING"
    subtitle = (
        "MuJoCo cable、接觸力、能量、機器人尺寸碰撞與 TCP 重播"
        if trajectory.physics_engine
        else "六自由度 IK、機器人尺寸碰撞、軟管接觸與自動 waypoint"
    )
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
    <section id="stage-motion" data-stage="C" class="pane motion-section reveal" aria-label="軟管夾取連續動作動畫">
      <header class="pane-header">
        <div class="pane-title"><span class="pane-code">C</span><div><h2>軟管抽取與搬運時間序列</h2><p>{escape(subtitle)}</p></div></div>
        <span class="pane-badge">{section_badge}</span>
      </header>
      <div class="phase-rail" aria-label="動作階段">{phase_rail}</div>
      <div class="motion-plot">{motion_html}</div>
      <div class="metric-grid">{motion_cards}</div>
      <details class="metric-disclosure">
        <summary>檢查全部動作與安全指標</summary>
        <div class="metric-table-wrap"><table class="metric-table">
          <thead><tr><th>運動指標</th><th>顯示值</th><th>資料欄位</th></tr></thead>
          <tbody>{_motion_metric_table(trajectory)}</tbody>
        </table></div>
      </details>
      <div class="motion-note">資料來源：{escape(engine_label)}。TCP 橘／紅色代表機器人低於 {trajectory.spec.safe_clearance_m * 1000.0:.1f} mm 或穿透；軟管節點橘色表示與管路進入 1 mm 接觸帶。規劃器插入的 waypoint 以橘色路徑節點顯示。</div>
    </section>
    """


def _analysis_metric_rows(
    metrics: dict[str, float | int],
    labels: dict[str, str],
) -> str:
    """建立物理、感知與整合區共用的完整指標列。"""

    rows = []
    for key, value in metrics.items():
        if key.endswith("_m"):
            display = f"{float(value) * 1000.0:.3f} mm"
        elif key.endswith("_deg"):
            display = f"{float(value):.3f}°"
        elif key.endswith("_ratio"):
            display = f"{float(value) * 100.0:.3f}%"
        elif key.endswith("_n"):
            display = f"{float(value):.3f} N"
        elif key.endswith("_j"):
            display = f"{float(value):.5f} J"
        elif key.endswith("_m_s"):
            display = f"{float(value):.3f} m/s"
        elif key.endswith("_s"):
            display = f"{float(value):.3f} s"
        else:
            display = f"{int(value):,}" if isinstance(value, int) else f"{float(value):.6f}"
        rows.append(
            "<tr>"
            f'<th scope="row">{escape(labels.get(key, key))}</th>'
            f"<td>{escape(display)}</td>"
            f"<td>{escape(key)}</td>"
            "</tr>"
        )
    return "".join(rows)


def _physics_section(
    kinematic: TrajectoryData | None,
    sweep: PhysicsSweepData | None,
) -> str:
    """建立 MuJoCo baseline、接觸物理與敏感度比較區。"""

    if kinematic is None or sweep is None:
        return ""
    figure = build_physics_comparison_figure(kinematic, sweep)
    physics_html = to_html(
        figure,
        include_plotlyjs=False,
        full_html=False,
        div_id="physics-view",
        config={"displaylogo": False, "responsive": True, "scrollZoom": True},
    )
    metrics = sweep.baseline.metrics
    cards = "".join(
        (
            f'<div class="metric {class_name}">'
            f'<span class="metric-label">{escape(label)}</span>'
            f'<span class="metric-value">{escape(value)}</span>'
            "</div>"
        )
        for label, value, class_name in (
            ("ENGINE", sweep.engine_version, "accent"),
            ("PHYSICS STEPS", f"{int(metrics['physics_step_count']):,}", ""),
            ("SAMPLED FORCE", f"{float(metrics['maximum_contact_force_n']):.2f} N", "warning"),
            ("GRASP LAG", f"{float(metrics['maximum_grasp_constraint_error_m']) * 1000.0:.2f} mm", "warning"),
            ("LENGTH ERROR", f"{float(metrics['maximum_hose_length_error_ratio']) * 100.0:.5f}%", "accent"),
            ("PENETRATION", f"{int(metrics['hose_penetration_frame_count'])}", "warning"),
        )
    )
    case_rows = "".join(
        "<tr>"
        f'<th scope="row">{escape(case.name)}</th>'
        f"<td>{case.parameters['timestep_s'] * 1000.0:.1f} ms</td>"
        f"<td>{case.parameters['bend_pa'] / 1.0e6:.2f} MPa</td>"
        f"<td>{case.parameters['friction']:.2f}</td>"
        f"<td>{float(case.metrics['final_shape_rms_delta_m']) * 1000.0:.2f} mm</td>"
        f"<td>{float(case.metrics['maximum_contact_force_n']):.2f} N</td>"
        "</tr>"
        for case in sweep.cases
    )
    labels = {
        "physics_step_count": "物理步進數",
        "maximum_physics_contact_count": "單幀環境接觸數",
        "maximum_contact_force_n": "抽樣最大環境接觸力",
        "minimum_contact_distance_m": "MuJoCo 最深環境接觸距離",
        "maximum_physics_self_contact_count": "單幀自接觸數",
        "maximum_self_contact_force_n": "抽樣最大自接觸力",
        "minimum_self_contact_distance_m": "最深自接觸距離",
        "maximum_grasp_constraint_error_m": "最大抓持約束誤差",
        "maximum_hose_speed_m_s": "輸出幀最大節點速度",
        "maximum_total_energy_j": "最大總能量",
        "physics_nonfinite_frame_count": "非有限值幀數",
    }
    physics_only_metrics = {
        key: value
        for key, value in metrics.items()
        if key.startswith(("physics_", "maximum_physics", "maximum_contact", "minimum_contact", "maximum_self", "minimum_self", "maximum_grasp", "maximum_total"))
        or key == "maximum_hose_speed_m_s"
    }
    return f"""
    <section id="stage-physics" data-stage="D" class="pane analysis-section reveal" aria-label="MuJoCo 軟管物理與敏感度">
      <header class="pane-header">
        <div class="pane-title"><span class="pane-code">D</span><div><h2>MuJoCo 軟管接觸物理</h2><p>cable 彎曲／扭轉、摩擦、接觸力、能量與 solver sensitivity</p></div></div>
        <span class="pane-badge">PHYSICS BASELINE</span>
      </header>
      <div class="motion-plot">{physics_html}</div>
      <div class="metric-grid">{cards}</div>
      <details class="metric-disclosure">
        <summary>檢查全部接觸與能量指標</summary>
        <div class="metric-table-wrap"><table class="metric-table">
          <thead><tr><th>物理指標</th><th>顯示值</th><th>資料欄位</th></tr></thead>
          <tbody>{_analysis_metric_rows(physics_only_metrics, labels)}</tbody>
        </table></div>
      </details>
      <div class="metric-table-wrap">
        <table class="metric-table"><thead><tr><th>敏感度案例</th><th>步長</th><th>彎曲參數</th><th>摩擦</th><th>最終形狀差</th><th>抽樣接觸力</th></tr></thead>
        <tbody>{case_rows}</tbody></table>
      </div>
      <div class="motion-note">接觸力只在 12 Hz 輸出幀抽樣；材料參數尚未由真實軟管校正。自接觸在此 baseline 關閉，避免相鄰 capsule 重疊造成假力。</div>
    </section>
    """


def _perception_section(
    sensor_result: RGBDSimulationResult,
    perception: PerceptionResult | None,
) -> str:
    """建立桌面、OBB、法向與抓取候選區。"""

    if perception is None:
        return ""
    figure = build_perception_figure(sensor_result.observation, perception)
    perception_html = to_html(
        figure,
        include_plotlyjs=False,
        full_html=False,
        div_id="perception-view",
        config={"displaylogo": False, "responsive": True, "scrollZoom": True},
    )
    metrics = perception.metrics
    cards = "".join(
        (
            f'<div class="metric {class_name}">'
            f'<span class="metric-label">{escape(label)}</span>'
            f'<span class="metric-value">{escape(value)}</span>'
            "</div>"
        )
        for label, value, class_name in (
            ("TABLE RMS", f"{float(metrics['table_plane_rms_error_m']) * 1000.0:.2f} mm", "warning"),
            ("TABLE HEIGHT", f"{float(metrics['table_height_error_m']) * 1000.0:+.2f} mm", ""),
            ("TABLE TILT", f"{float(metrics['table_tilt_error_deg']):.2f}°", "accent"),
            ("OBJECTS", f"{int(metrics['detected_object_count'])}", "accent"),
            ("GRASPS", f"{int(metrics['grasp_candidate_count'])}", ""),
            ("FEASIBLE", f"{int(metrics['feasible_grasp_candidate_count'])}", "accent"),
        )
    )
    labels = {
        "valid_point_count": "Observation 有效點",
        "table_inlier_count": "桌面內點數",
        "table_inlier_ratio": "桌面內點比例",
        "table_plane_rms_error_m": "桌面平面 RMS",
        "table_tilt_error_deg": "桌面傾角誤差",
        "table_height_error_m": "桌面高度誤差",
        "detected_object_count": "物件數",
        "grasp_candidate_count": "抓取候選數",
        "feasible_grasp_candidate_count": "幾何可行候選數",
    }
    return f"""
    <section id="stage-perception" data-stage="E" class="pane analysis-section reveal" aria-label="RGB-D 幾何與抓取候選">
      <header class="pane-header">
        <div class="pane-title"><span class="pane-code">E</span><div><h2>RGB-D 幾何與抓取候選</h2><p>RANSAC 桌面、oracle instance baseline、AABB／OBB、法向與 top-down grasp</p></div></div>
        <span class="pane-badge">PERCEPTION BASELINE</span>
      </header>
      <div class="motion-plot">{perception_html}</div>
      <div class="metric-grid">{cards}</div>
      <details class="metric-disclosure">
        <summary>檢查全部桌面與抓取幾何指標</summary>
        <div class="metric-table-wrap"><table class="metric-table">
          <thead><tr><th>感知指標</th><th>顯示值</th><th>資料欄位</th></tr></thead>
          <tbody>{_analysis_metric_rows(metrics, labels)}</tbody>
        </table></div>
      </details>
      <div class="motion-note">物件分割目前使用模擬 instance mask 作為 oracle baseline；桌面、包圍盒、法向與抓取幾何則由含雜訊 observation 計算，尚未宣稱未知物件分割能力。</div>
    </section>
    """


def _integration_section(replay: ReplayResult | None) -> str:
    """建立 fail-closed 安全閘門與控制重播摘要區。"""

    if replay is None:
        return ""
    status = "AUTHORIZED" if replay.execution_authorized else "ABORTED"
    status_class = "accent" if replay.execution_authorized else "warning"
    gate_class = "" if replay.execution_authorized else " is-fault"
    selected_name = (
        "NONE"
        if replay.selected_grasp is None
        else replay.selected_grasp.candidate.object_name
    )
    cards = "".join(
        (
            f'<div class="metric {class_name}">'
            f'<span class="metric-label">{escape(label)}</span>'
            f'<span class="metric-value">{escape(value)}</span>'
            "</div>"
        )
        for label, value, class_name in (
            ("EXECUTION", status, status_class),
            ("FAILURES", f"{len(replay.failure_codes)}", status_class),
            ("SELECTED", selected_name, ""),
            ("EVENTS", f"{len(replay.events)}", ""),
            ("COMMANDS", f"{int(replay.metrics['command_frame_count'])}", "accent"),
            ("DURATION", f"{float(replay.metrics['replay_duration_s']):.1f} s", ""),
        )
    )
    failure_text = "PASS — 全部安全閘門通過" if not replay.failure_codes else "、".join(replay.failure_codes)
    labels = {
        "execution_authorized": "允許執行",
        "failure_count": "失敗分類數",
        "event_count": "事件數",
        "command_frame_count": "控制命令幀數",
        "replay_duration_s": "重播時間",
        "validated_grasp_count": "完成 IK／碰撞驗證的抓取數",
        "selected_grasp_score": "選定抓取分數",
        "selected_grasp_clearance_m": "選定抓取最小距離",
    }
    return f"""
    <section id="stage-control" data-stage="F" class="pane analysis-section reveal" aria-label="安全閘門與控制重播">
      <header class="pane-header">
        <div class="pane-title"><span class="pane-code">F</span><div><h2>Fail-closed 規劃與控制重播</h2><p>感知候選 → IK → 碰撞 → 物理門檻 → JSONL 命令事件</p></div></div>
        <span class="pane-badge">{status}</span>
      </header>
      <div class="gate-banner{gate_class}"><span>COMMAND GATE / {escape(failure_text)}</span><strong>{status}</strong></div>
      <div class="metric-grid">{cards}</div>
      <details class="metric-disclosure">
        <summary>檢查全部安全閘門與事件指標</summary>
        <div class="metric-table-wrap"><table class="metric-table">
          <thead><tr><th>整合指標</th><th>顯示值</th><th>資料欄位</th></tr></thead>
          <tbody>{_analysis_metric_rows(replay.metrics, labels)}</tbody>
        </table></div>
      </details>
      <div class="motion-note">安全閘門：{escape(failure_text)}。此 JSONL 是 message-neutral 離線重播，不會連線或命令真實機器；URDF／SRDF 僅含目前簡化幾何，接入 ROS 2／MoveIt 前仍需控制器與實機安全設定。</div>
    </section>
    """


def write_simulation_report(
    scene_data: SceneData,
    result: RGBDSimulationResult,
    output_path: str | Path,
    trajectory: TrajectoryData | None = None,
    physics_sweep: PhysicsSweepData | None = None,
    perception: PerceptionResult | None = None,
    replay: ReplayResult | None = None,
) -> Path:
    """輸出世界、感測、動作、物理、感知與安全整合的單頁報告。"""

    destination = Path(output_path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    scene_figure = build_figure(scene_data)
    scene_figure.update_layout(
        title=None,
        height=650,
        margin={"l": 0, "r": 0, "t": 12, "b": 0},
        paper_bgcolor=CERAMIC,
    )
    sensor_figure = build_rgbd_comparison_figure(result)
    sensor_figure.update_layout(
        title=None,
        height=650,
        margin={"l": 28, "r": 70, "t": 54, "b": 28},
        paper_bgcolor=CERAMIC,
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
    displayed_trajectory = (
        physics_sweep.baseline if physics_sweep is not None else trajectory
    )
    motion_section = _motion_section(scene_data, displayed_trajectory)
    physics_section = _physics_section(trajectory, physics_sweep)
    perception_section = _perception_section(result, perception)
    integration_section = _integration_section(replay)
    motion_resize = (
        'Plotly.Plots.resize(document.getElementById("motion-view"));'
        if displayed_trajectory is not None
        else ""
    )
    physics_resize = (
        'Plotly.Plots.resize(document.getElementById("physics-view"));'
        if trajectory is not None and physics_sweep is not None
        else ""
    )
    perception_resize = (
        'Plotly.Plots.resize(document.getElementById("perception-view"));'
        if perception is not None
        else ""
    )
    stages = [
        ("A", "世界", "#stage-world"),
        ("B", "感測", "#stage-sensor"),
    ]
    if displayed_trajectory is not None:
        stages.append(("C", "動作", "#stage-motion"))
    if trajectory is not None and physics_sweep is not None:
        stages.append(("D", "物理", "#stage-physics"))
    if perception is not None:
        stages.append(("E", "抓取", "#stage-perception"))
    if replay is not None:
        stages.append(("F", "閘門", "#stage-control"))
    stage_navigation = "".join(
        f'<a href="{target}" data-nav-stage="{code}"><span>{code}</span>{label}</a>'
        for code, label, target in stages
    )
    physics_signal = "CABLE ACTIVE" if physics_sweep is not None else "NOT RUN"
    physics_signal_class = " is-live" if physics_sweep is not None else ""
    if replay is None:
        command_signal = "NOT EVALUATED"
        command_signal_class = ""
    elif replay.execution_authorized:
        command_signal = "AUTHORIZED"
        command_signal_class = " is-live"
    else:
        command_signal = "ABORTED"
        command_signal_class = " is-fault"
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
  <a class="skip-link" href="#report-main">跳到模擬結果</a>
  <header class="instrument-bar">
    <div class="instrument-grid">
      <div>
        <p class="eyebrow">SimGrasp3D / simulation validation</p>
        <h1>從世界座標到安全命令</h1>
      </div>
      <div class="run-state" aria-label="執行識別">
        <div class="state-cell"><span class="meta-label">Scene</span><span class="meta-value">{escape(spec.name)}</span></div>
        <div class="state-cell"><span class="meta-label">Seed</span><span class="meta-value">{spec.seed}</span></div>
        <div class="state-cell"><span class="meta-label">Entities</span><span class="meta-value">{len(scene_data.point_clouds)}</span></div>
        <div class="state-cell"><span class="meta-label">Points</span><span class="meta-value">{total_points:,}</span></div>
      </div>
    </div>
    <div class="signal-path" aria-label="模擬資料處理鏈">
      <span class="signal-pulse" aria-hidden="true"></span>
      <div class="signal-node is-live"><span>01 / WORLD</span><strong>GEOMETRY READY</strong></div>
      <div class="signal-node is-live"><span>02 / SENSOR</span><strong>RGB-D OBSERVED</strong></div>
      <div class="signal-node{physics_signal_class}"><span>03 / PHYSICS</span><strong>{physics_signal}</strong></div>
      <div class="signal-node{command_signal_class}"><span>04 / COMMAND</span><strong>{command_signal}</strong></div>
    </div>
  </header>
  <nav class="chapter-nav" style="--stage-count: {len(stages)}" aria-label="驗證階段導覽">
    {stage_navigation}
  </nav>
  <main id="report-main">
    <section class="context-strip reveal" aria-label="測試情境與條件">
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

    <section id="stage-world" data-stage="A" class="workbench reveal" aria-label="原始場景與感測結果雙畫面比較">
      <div class="calibration-rail"><span class="rail-label">WORLD → SENSOR</span></div>
      <article class="pane world">
        <header class="pane-header">
          <div class="pane-title"><span class="pane-code">A</span><div><h2>原始模擬世界</h2><p>完整幾何、機械手、TCP、座標系與相機視錐</p></div></div>
          <span class="pane-badge">GROUND TRUTH WORLD</span>
        </header>
        <div class="plot-shell">{scene_html}</div>
      </article>
      <article id="stage-sensor" class="pane sensor">
        <header class="pane-header">
          <div class="pane-title"><span class="pane-code">B</span><div><h2>相機測試結果</h2><p>理想深度、含誤差觀測、絕對誤差與 RGB</p></div></div>
          <span class="pane-badge">SENSOR OBSERVATION</span>
        </header>
        <div class="plot-shell">{sensor_html}</div>
      </article>
    </section>

    <section class="results reveal" aria-label="全部量化測試結果">
      <header class="results-header"><h2>量化結果</h2><p>深度誤差只在 ground truth 與 observation 的共同有效像素計算</p></header>
      <div class="metric-grid">{metric_cards}</div>
      <details class="metric-disclosure">
        <summary>檢查全部 RGB-D 量測指標</summary>
        <div class="metric-table-wrap"><table class="metric-table">
          <thead><tr><th>指標</th><th>顯示值</th><th>資料欄位</th></tr></thead>
          <tbody>{_metric_table(result)}</tbody>
        </table></div>
      </details>
    </section>
    {motion_section}
    {physics_section}
    {perception_section}
    {integration_section}
    <p class="report-note reveal">此頁是模擬結果，不是實機驗證。較大的深度邊界誤差會同時包含外參偏移後不同表面落入同一像素的影響；目前的影像填充率也受離散表面點數影響。</p>
  </main>
  <script>
    document.body.classList.add("motion-ready");
    const reducedMotion = window.matchMedia("(prefers-reduced-motion: reduce)").matches;
    const revealItems = document.querySelectorAll(".reveal");
    if (reducedMotion || !("IntersectionObserver" in window)) {{
      revealItems.forEach(function (item) {{ item.classList.add("is-visible"); }});
    }} else {{
      const revealObserver = new IntersectionObserver(function (entries, observer) {{
        entries.forEach(function (entry) {{
          if (entry.isIntersecting) {{
            entry.target.classList.add("is-visible");
            observer.unobserve(entry.target);
          }}
        }});
      }}, {{ rootMargin: "0px 0px -10% 0px", threshold: 0.08 }});
      revealItems.forEach(function (item) {{ revealObserver.observe(item); }});
    }}

    const navLinks = document.querySelectorAll("[data-nav-stage]");
    function activateStage(stage) {{
      navLinks.forEach(function (link) {{
        const active = link.dataset.navStage === stage;
        link.classList.toggle("is-active", active);
        if (active) {{
          link.setAttribute("aria-current", "step");
        }} else {{
          link.removeAttribute("aria-current");
        }}
      }});
    }}
    if (navLinks.length) {{ activateStage(navLinks[0].dataset.navStage); }}
    navLinks.forEach(function (link) {{
      link.addEventListener("click", function () {{ activateStage(link.dataset.navStage); }});
    }});
    if ("IntersectionObserver" in window) {{
      const stageObserver = new IntersectionObserver(function (entries) {{
        const visible = entries
          .filter(function (entry) {{ return entry.isIntersecting; }})
          .sort(function (left, right) {{ return right.intersectionRatio - left.intersectionRatio; }});
        if (visible.length) {{ activateStage(visible[0].target.dataset.stage); }}
      }}, {{ rootMargin: "-15% 0px -62% 0px", threshold: [0.08, 0.25] }});
      document.querySelectorAll("[data-stage]").forEach(function (item) {{
        stageObserver.observe(item);
      }});
    }}

    window.addEventListener("load", function () {{
      window.setTimeout(function () {{
        Plotly.Plots.resize(document.getElementById("world-view"));
        Plotly.Plots.resize(document.getElementById("sensor-view"));
        {motion_resize}
        {physics_resize}
        {perception_resize}
      }}, 80);
    }});
  </script>
</body>
</html>
"""
    destination.write_text(document, encoding="utf-8")
    return destination
