"""建立 SimGrasp3D 的統一學習入口與本次執行摘要。"""

from __future__ import annotations

from html import escape
from pathlib import Path
from typing import TYPE_CHECKING

from simgrasp3d.visualization.assets import read_asset

if TYPE_CHECKING:
    from simgrasp3d.models.hospital import HospitalSuiteResult
    from simgrasp3d.models.integration import ReplayResult
    from simgrasp3d.models.motion import TrajectoryData
    from simgrasp3d.models.perception import PerceptionResult
    from simgrasp3d.models.physics import PhysicsSweepData
    from simgrasp3d.models.system_design import SystemDesignLabResult
    from simgrasp3d.scene.builder import SceneData
    from simgrasp3d.sensors.rgbd import RGBDSimulationResult


_CSS = read_asset("home.css")


_JS = read_asset("home.js")


def _safe_metric(source: object | None, key: str, default: float = 0.0) -> float:
    if source is None:
        return default
    metrics = getattr(source, "metrics", {})
    return float(metrics.get(key, default))


def _rail_node(code: str, label: str, state: str, ready: bool, fault: bool = False) -> str:
    class_name = "rail-node fault" if fault else "rail-node ready" if ready else "rail-node"
    return (
        f'<div class="{class_name}"><span class="rail-label">{escape(code)} / {escape(label)}</span>'
        f'<strong>{escape(state)}</strong></div>'
    )


def _station(
    *,
    code: str,
    title: str,
    description: str,
    href: str | None,
    status: str,
    class_name: str,
    tags: tuple[str, ...],
) -> str:
    tag_html = "".join(f"<span>{escape(tag)}</span>" for tag in tags)
    classes = f"station {class_name}" + (" unavailable" if href is None else "")
    tag = "a" if href is not None else "div"
    href_attr = f' href="{escape(href)}"' if href is not None else ""
    status_class = "status" if href is not None else "status idle"
    return (
        f'<{tag} class="{classes}"{href_attr}><div class="station-copy">'
        f'<span class="station-code">{escape(code)}</span><h3>{escape(title)}</h3>'
        f'<p>{escape(description)}</p><div class="station-meta">{tag_html}</div></div>'
        f'<span class="{status_class}">{escape(status)}</span><span class="station-arrow">→</span></{tag}>'
    )


def write_home_dashboard(
    output_path: str | Path,
    scene: SceneData,
    *,
    design: SystemDesignLabResult | None = None,
    sensor: RGBDSimulationResult | None = None,
    trajectory: TrajectoryData | None = None,
    physics: PhysicsSweepData | None = None,
    perception: PerceptionResult | None = None,
    replay: ReplayResult | None = None,
    hospital: HospitalSuiteResult | None = None,
    design_href: str | None = None,
    report_href: str | None = None,
    hospital_href: str | None = None,
) -> Path:
    """輸出專案主入口，內容由本次實際執行結果生成。"""

    destination = Path(output_path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    design_passed = int(design.baseline.metrics["passed_gate_count"]) if design else 0
    design_total = int(design.baseline.metrics["gate_count"]) if design else 0
    hospital_count = len(hospital.cases) if hospital else 0
    report_layers = sum(item is not None for item in (sensor, trajectory, physics, perception, replay))
    stations = "".join(
        (
            _station(
                code="START / DESIGN",
                title="系統設計實驗室",
                description="調整相機、六軸手臂、夾爪、軟管與障礙包絡，立即看出哪個子系統先失敗。從這裡開始建立設計直覺。",
                href=design_href,
                status=f"{design_passed}/{design_total} GATES" if design else "NOT RUN",
                class_name="station-primary",
                tags=("12 個參數", "視錐", "工作空間", "路徑淨空"),
            ),
            _station(
                code="VERIFY / A–F",
                title="完整模擬驗證",
                description="檢查 RGB-D、IK、碰撞、MuJoCo、3D 感知與 fail-closed 命令證據。",
                href=report_href,
                status=f"{report_layers}/5 LAYERS" if sensor else "NOT RUN",
                class_name="station-report",
                tags=("Ground truth", "Observation", "Physics"),
            ),
            _station(
                code="TRANSFER / H1–H7",
                title="醫院情境研究",
                description="把相同方法帶入試管、器械盤、床旁管路、配送、消毒與假體案例。",
                href=hospital_href,
                status=f"{hospital_count} CASES" if hospital else "NOT RUN",
                class_name="station-hospital",
                tags=("分頁案例", "風險界線", "同步重播"),
            ),
        )
    )

    sensor_ready = sensor is not None
    motion_ready = trajectory is not None
    physics_ready = physics is not None
    command_ready = replay is not None and replay.execution_authorized
    command_fault = replay is not None and not replay.execution_authorized
    rail = "".join(
        (
            _rail_node("D0", "DESIGN", f"{design_passed}/{design_total} GATES" if design else "NOT RUN", design is not None),
            _rail_node("D1", "WORLD", f"{len(scene.point_clouds)} ENTITIES", True),
            _rail_node("D2", "SENSOR", "RGB-D READY" if sensor_ready else "NOT RUN", sensor_ready),
            _rail_node("D3", "PLAN", "TRAJECTORY READY" if motion_ready else "NOT RUN", motion_ready),
            _rail_node("D4", "PHYSICS", "MUJOCO READY" if physics_ready else "NOT RUN", physics_ready),
            _rail_node("D5", "COMMAND", "AUTHORIZED" if command_ready else "ABORTED" if command_fault else "NOT RUN", command_ready, command_fault),
        )
    )

    depth_mae = _safe_metric(sensor, "depth_mae_m") * 1000.0
    ik_error = _safe_metric(trajectory, "maximum_ik_error_m") * 1000.0
    clearance = _safe_metric(trajectory, "minimum_robot_clearance_m") * 1000.0
    contact_force = _safe_metric(physics.baseline if physics else None, "maximum_contact_force_n")
    grasps = int(_safe_metric(perception, "feasible_grasp_candidate_count"))
    command_text = "AUTHORIZED" if command_ready else "ABORTED" if command_fault else "NOT RUN"
    metrics = (
        ("DEPTH MAE", f"{depth_mae:.2f} mm" if sensor else "—", "觀測與真值共同有效像素", "warning" if sensor else ""),
        ("MAX IK ERROR", f"{ik_error:.2f} mm" if trajectory else "—", "六自由度姿態軌跡", "live" if trajectory else ""),
        ("MIN CLEARANCE", f"{clearance:.2f} mm" if trajectory else "—", "機器人外形到固定障礙", "live" if trajectory else ""),
        ("CONTACT FORCE", f"{contact_force:.2f} N" if physics else "—", "12 Hz 輸出幀抽樣值", "warning" if physics else ""),
        ("FEASIBLE GRASP", str(grasps) if perception else "—", "幾何可行候選數", "live" if perception and grasps > 0 else ""),
        ("COMMAND GATE", command_text, "離線重播，不連接實機", "live" if command_ready else "fault" if command_fault else ""),
    )
    metric_html = "".join(
        f'<div class="metric {state}"><span class="metric-label">{escape(label)}</span><strong>{escape(value)}</strong><p>{escape(note)}</p></div>'
        for label, value, note, state in metrics
    )

    primary_design_href = escape(design_href or "#stations")
    primary_report_href = escape(report_href or "#snapshot")
    sensor_stage_href = escape(f"{report_href}#stage-sensor" if report_href and sensor else "#snapshot")
    grasp_stage_href = escape(
        f"{report_href}#stage-perception" if report_href and perception else "#snapshot"
    )
    motion_stage_href = escape(
        f"{report_href}#stage-motion" if report_href and trajectory else "#snapshot"
    )
    control_stage_href = escape(
        f"{report_href}#stage-control" if report_href and replay else "#snapshot"
    )
    document = f'''<!doctype html>
<html lang="zh-Hant"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>SimGrasp3D Lab｜3D 抓取系統設計學習</title><style>{_CSS}</style></head>
<body><a class="skip" href="#main">跳到學習入口</a>
<header class="topbar"><span class="brand">SIMGRASP3D LAB / LEARNING CONSOLE</span><span class="run-id">SCENE {escape(scene.spec.name)} · SEED {scene.spec.seed}</span></header>
<section class="hero"><div class="hero-copy"><p class="eyebrow">3D perception × manipulation × flexible object</p><h1>把一套抓取系統，<br>拆成可以驗證的決策。</h1><p>從相機看不看得到、手臂到不到得了，到夾爪能否抓住軟管、路徑是否會碰撞。先調參建立假設，再用完整模擬留下證據。</p>
<div class="hero-actions"><a class="cta primary" href="{primary_design_href}">開始設計系統 →</a><a class="cta" href="{primary_report_href}">查看本次驗證</a></div>
<div class="run-stamp"><b>{'RUN COMPLETE' if sensor_ready and motion_ready else 'PARTIAL RUN'}</b><span>這是固定 seed 的 simulation-only 學習資料；通過教學門檻不等於實機或醫療安全驗證。</span></div></div>
<div class="schematic" aria-label="相機、手臂、軟管與規劃路徑系統示意">
<svg viewBox="0 0 720 470" role="img" aria-labelledby="diagram-title diagram-desc"><title id="diagram-title">軟管抓取系統校正圖</title><desc id="diagram-desc">固定相機觀察軟管，六軸手臂沿規劃路徑避開三組管路障礙。</desc>
<defs><pattern id="grid" width="24" height="24" patternUnits="userSpaceOnUse"><path d="M24 0H0V24" fill="none" stroke="#6b8a91" stroke-width="1"/></pattern></defs><rect class="blueprint-grid" width="720" height="470" fill="url(#grid)"/>
<path d="M78 392H652" stroke="#718990" stroke-width="2"/><path d="M92 392V110" stroke="#49636b" stroke-width="1" stroke-dasharray="5 8"/>
<g class="system-node" tabindex="0" data-title="RGB-D 與 camera→world" data-copy="先檢查視錐、遮蔽、工作距離，再把深度與外參誤差轉成抓取容差。"><path class="node-main" d="M534 75h82v52h-82z" fill="#24454e" stroke="#72d4d1" stroke-width="2"/><circle cx="550" cy="101" r="10" fill="none" stroke="#72d4d1" stroke-width="3"/><path d="M534 125L282 354 616 127Z" fill="#058b91" fill-opacity=".09" stroke="#4ca6a7" stroke-dasharray="7 7"/><text x="575" y="104" fill="#dff7f4" font-size="11" text-anchor="middle">RGB-D</text><path class="node-hit" d="M520 60h110v85H520z"/></g>
<g class="system-node" tabindex="0" data-title="六軸手臂與 TCP" data-copy="連桿尺寸、關節限制、TCP offset 與夾爪外形共同決定可達性和碰撞包絡。"><rect class="node-main" x="104" y="330" width="78" height="62" fill="#263f47" stroke="#d68d22" stroke-width="2"/><path d="M143 330L188 271 261 290 320 223 383 248" fill="none" stroke="#d6e2df" stroke-width="13" stroke-linecap="round" stroke-linejoin="round"/><g fill="#d68d22" stroke="#fff"><circle cx="143" cy="330" r="8"/><circle cx="188" cy="271" r="8"/><circle cx="261" cy="290" r="8"/><circle cx="320" cy="223" r="8"/><circle cx="383" cy="248" r="8"/></g><path d="M383 248h37m-4-13v26" stroke="#d68d22" stroke-width="6"/><path class="node-hit" d="M90 205h345v195H90z"/></g>
<g class="system-node" tabindex="0" data-title="軟管中心線與夾取點" data-copy="管徑、材料允許彎曲半徑、夾取位置與抓持力必須分開建模和校正。"><path class="node-main" d="M230 366C290 330 333 379 389 349S492 326 570 360" fill="none" stroke="#6ed5d1" stroke-width="11" stroke-linecap="round"/><circle cx="390" cy="349" r="11" fill="#d68d22" stroke="#fff" stroke-width="3"/><path class="node-hit" d="M215 314h370v75H215z"/></g>
<g class="system-node" tabindex="0" data-title="固定管路與安全包絡" data-copy="障礙幾何還要加上定位誤差、工具尺寸與安全距離，才能成為規劃使用的碰撞模型。"><rect class="node-main" x="443" y="208" width="25" height="153" fill="#526b73" stroke="#9db2b7" stroke-width="2"/><rect x="470" y="270" width="147" height="24" fill="#526b73" stroke="#9db2b7" stroke-width="2"/><rect x="256" y="318" width="23" height="74" fill="#526b73" stroke="#9db2b7" stroke-width="2"/><path class="node-hit" d="M245 194h390v205H245z"/></g>
<g class="system-node" tabindex="0" data-title="TCP 路徑與 waypoint" data-copy="先做工作空間篩選，再檢查姿態 IK、連續碰撞和被軟管掃過的體積。"><path class="node-main" d="M420 248C445 189 512 174 554 215S572 303 591 330" fill="none" stroke="#e0a13c" stroke-width="4" stroke-dasharray="10 7"/><circle cx="512" cy="180" r="7" fill="#e0a13c"/><path class="scan" d="M280 84V390" stroke="#7ce1dc" stroke-width="2" opacity=".6"/><path class="node-hit" d="M404 158h210v190H404z"/></g>
<g fill="#8fa8ae" font-family="monospace" font-size="10"><text x="97" y="105">Z / WORLD</text><text x="611" y="410">X / WORLD</text><text x="115" y="423">ROBOT BASE</text><text x="530" y="66">EYE-TO-HAND</text><text x="288" y="414">HOSE + OBSTACLES</text></g></svg>
<div class="schematic-readout" id="schematic-readout"><span class="schematic-label">SYSTEM LAYER / 移入圖面查看</span><div><strong>一個世界座標，五種可失敗的模型</strong><span>把游標移到相機、手臂、軟管、障礙或路徑，查看每一層必須設計的內容。</span></div></div></div></section>
<div class="system-rail" aria-label="本次執行狀態">{rail}</div>
<main class="main" id="main"><section id="stations"><header class="section-head"><div><h2>選擇你的下一個實驗</h2></div><p>如果還不知道參數如何互相影響，先進系統設計實驗室；已有假設後，再進完整報告檢查數據與物理證據。</p></header><div class="stations">{stations}</div></section>
<section class="snapshot" id="snapshot"><header class="snapshot-head"><h2>本次執行快照</h2><span>ACTUAL OUTPUT / FIXED SEED {scene.spec.seed}</span></header><div class="metrics">{metric_html}</div></section>
<section class="curriculum"><header class="section-head"><div><h2>從情境到可信結果</h2></div><p>順序不是頁面裝飾，而是依賴關係：下游通過不能補救上游使用錯誤座標或不可見資料。</p></header><div class="phases">
<article class="phase"><span class="phase-code">01 / FRAME</span><h3>建立尺寸與座標</h3><p>用 meter 定義 base、TCP、camera、桌面、軟管及固定障礙。</p><a href="{primary_design_href}">調整硬體參數 →</a></article>
<article class="phase"><span class="phase-code">02 / OBSERVE</span><h3>模擬真正看見的資料</h3><p>世界點經 pinhole、z-buffer、量化、孔洞與外參誤差成為 RGB-D。</p><a href="{sensor_stage_href}">查看感測比較 →</a></article>
<article class="phase"><span class="phase-code">03 / GRASP</span><h3>從點雲產生候選</h3><p>分割桌面與物件，建立 OBB、法向、pregrasp 與 grasp pose。</p><a href="{grasp_stage_href}">查看抓取分析 →</a></article>
<article class="phase"><span class="phase-code">04 / PLAN</span><h3>求解 IK 與避障</h3><p>檢查關節限制、機器人外形、工具包絡和軟管抽取路徑。</p><a href="{motion_stage_href}">查看連續動作 →</a></article>
<article class="phase"><span class="phase-code">05 / VERIFY</span><h3>物理與安全停止</h3><p>用 MuJoCo、敏感度與 fail-closed 閘門決定是否產生離線命令。</p><a href="{control_stage_href}">查看安全閘門 →</a></article>
</div></section>
<section class="scope"><div><h2>這一版可以學到什麼</h2><p>設定、觀測、規劃、物理與安全層之間的資料契約，以及一個參數改變後應該在哪個量測值留下證據。</p><ul><li>相機位置與視錐如何影響軟管可見率。</li><li>手臂尺寸、TCP 與抬升高度如何影響工作空間。</li><li>管徑、夾爪開口與彎曲限制如何影響抓取。</li><li>工具包絡、安全距離與 waypoint 如何影響路徑。</li></ul></div><div><h2>這一版不能證明什麼</h2><p>模擬畫面流暢不代表模型正確。本專案尚未提供實機控制、材料校正、真實相機 adapter、醫療器材符合性或臨床安全證據。</p><ul><li>教學門檻全部未校準。</li><li>瀏覽器工作台是快速幾何估算器。</li><li>接觸力是輸出幀抽樣，不是完整峰值認證。</li><li>所有控制命令只寫入離線重播檔。</li></ul></div></section></main>
<footer class="footer"><div class="footer-inner"><span>SimGrasp3D Lab v0.12 · author zack7515 · simulation-only</span><span><a href="../README.md">README</a> · Python / NumPy / Plotly / MuJoCo</span></div></footer>
<script>{_JS}</script></body></html>'''
    destination.write_text(document, encoding="utf-8")
    return destination
