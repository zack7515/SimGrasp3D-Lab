"""建立 SimGrasp3D 的統一學習入口與本次執行摘要。"""

from __future__ import annotations

from html import escape
from pathlib import Path
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from simgrasp3d.models.hospital import HospitalSuiteResult
    from simgrasp3d.models.integration import ReplayResult
    from simgrasp3d.models.motion import TrajectoryData
    from simgrasp3d.models.perception import PerceptionResult
    from simgrasp3d.models.physics import PhysicsSweepData
    from simgrasp3d.models.system_design import SystemDesignLabResult
    from simgrasp3d.scene.builder import SceneData
    from simgrasp3d.sensors.rgbd import RGBDSimulationResult


_CSS = r"""
:root{--blueprint:#14272d;--titanium:#203a42;--ink:#17262b;--slate:#65777d;--paper:#f3f6f3;--panel:#fbfcfa;--line:#c8d5d1;--grid:#dce6e2;--measure:#058b91;--measure-soft:#d9eeee;--amber:#d68d22;--amber-soft:#f7e8ca;--fault:#c64d3f;--fault-soft:#f5ddda;--violet:#715b91;--display:"Bahnschrift SemiCondensed","DIN Alternate","Arial Narrow",sans-serif;--body:Aptos,"Noto Sans TC","PingFang TC","Microsoft JhengHei",sans-serif;--mono:"JetBrains Mono","IBM Plex Mono",Consolas,monospace}
*{box-sizing:border-box}html{scroll-behavior:smooth}body{margin:0;color:var(--ink);background:var(--paper);font-family:var(--body);line-height:1.5}a{color:inherit}:focus-visible{outline:3px solid var(--amber);outline-offset:3px}.skip{position:fixed;z-index:100;top:8px;left:8px;padding:8px 12px;background:#fff;transform:translateY(-180%)}.skip:focus{transform:none}
.topbar{position:sticky;z-index:40;top:0;display:flex;justify-content:space-between;align-items:center;min-height:50px;padding:0 clamp(16px,4vw,56px);color:#eef7f5;background:rgba(20,39,45,.97);border-bottom:1px solid #486169;backdrop-filter:blur(10px)}.brand,.run-id,.eyebrow,.station-code,.status,.metric-label,.phase-code,.rail-label,.schematic-label{font-family:var(--mono);text-transform:uppercase;letter-spacing:.09em}.brand{font-size:10px}.run-id{color:#91a7ad;font-size:9px}
.hero{display:grid;grid-template-columns:minmax(0,1.02fr) minmax(520px,.98fr);min-height:580px;color:#f1f8f6;background:var(--blueprint)}.hero-copy{align-self:center;padding:clamp(44px,7vw,104px)}.eyebrow{margin:0 0 16px;color:#72d4d1;font-size:10px}.hero h1{max-width:840px;margin:0;font-family:var(--display);font-size:clamp(50px,6.7vw,98px);font-weight:700;letter-spacing:-.04em;line-height:.85}.hero-copy>p:not(.eyebrow){max-width:720px;margin:26px 0 0;color:#adc0c4;font-size:14px}.hero-actions{display:flex;flex-wrap:wrap;gap:8px;margin-top:28px}.cta{display:inline-flex;align-items:center;min-height:44px;padding:10px 14px;border:1px solid #628087;color:#fff;background:transparent;font-family:var(--mono);font-size:9px;letter-spacing:.06em;text-decoration:none}.cta.primary{color:var(--blueprint);background:#78d7d2;border-color:#78d7d2}.cta:hover{border-color:#fff}.run-stamp{display:grid;grid-template-columns:auto 1fr;gap:12px;align-items:center;margin-top:42px;padding-top:14px;border-top:1px solid #3d555d;color:#8fa7ad;font-size:9px}.run-stamp b{color:#80dcd6;font-family:var(--mono);font-size:9px;letter-spacing:.08em}
.schematic{position:relative;overflow:hidden;display:grid;grid-template-rows:1fr auto;min-width:0;padding:30px 28px 24px;background:#193139;border-left:1px solid #49616a}.schematic svg{align-self:center;width:100%;height:auto;max-height:470px}.blueprint-grid{opacity:.25}.system-node{cursor:help;outline:none}.system-node .node-hit{display:none}.system-node:hover .node-main,.system-node:focus .node-main{stroke:#fff;stroke-width:4}.scan{animation:scan 5s ease-in-out infinite}.schematic-readout{display:grid;grid-template-columns:130px 1fr;gap:16px;min-height:67px;padding:13px 0 0;border-top:1px solid #49616a;color:#9db2b7;font-size:10px}.schematic-label{color:#71d1cd;font-size:8px}.schematic-readout strong{display:block;color:#edf7f5;font-size:12px}.schematic-readout span:last-child{display:block;margin-top:3px}@keyframes scan{0%,100%{transform:translateX(-30px);opacity:.18}50%{transform:translateX(250px);opacity:.75}}
.system-rail{display:grid;grid-template-columns:repeat(6,1fr);color:#dce8e6;background:var(--titanium);border-bottom:1px solid #50676f}.rail-node{position:relative;min-width:0;padding:13px 15px;border-right:1px solid #496169}.rail-node:last-child{border-right:0}.rail-node:before{content:"";position:absolute;top:0;right:0;left:0;height:3px;background:#71858b}.rail-node.ready:before{background:var(--measure)}.rail-node.fault:before{background:var(--fault)}.rail-label{display:block;color:#849ba2;font-size:8px}.rail-node strong{display:block;overflow:hidden;margin-top:4px;font-size:10px;text-overflow:ellipsis;white-space:nowrap}.rail-node.ready strong{color:#8ae0dc}.rail-node.fault strong{color:#ffaaa2}
.main{width:min(1480px,100%);margin:0 auto;padding:46px clamp(14px,4vw,56px) 72px}.section-head{display:flex;justify-content:space-between;gap:28px;align-items:end;margin-bottom:15px}.section-head h2{margin:0;font-family:var(--display);font-size:30px;letter-spacing:-.015em}.section-head p{max-width:620px;margin:0;color:var(--slate);font-size:11px}.stations{display:grid;grid-template-columns:1.25fr .75fr;grid-template-rows:1fr 1fr;gap:12px}.station{position:relative;overflow:hidden;display:grid;grid-template-columns:1fr auto;min-height:196px;padding:23px;color:inherit;background:var(--panel);border:1px solid var(--line);text-decoration:none;transition:transform .18s,box-shadow .18s,border-color .18s}.station-primary{grid-row:1/3;min-height:404px;color:#edf7f5;background:var(--titanium);border-color:#49636b}.station:before{content:"";position:absolute;top:0;bottom:0;left:0;width:5px;background:var(--measure)}.station-report:before{background:var(--amber)}.station-hospital:before{background:var(--violet)}.station:hover{z-index:2;transform:translateY(-3px);border-color:#6aa8a8;box-shadow:0 18px 35px rgba(20,39,45,.12)}.station-copy{align-self:end;max-width:690px}.station-code{color:var(--measure);font-size:9px}.station-primary .station-code{color:#79d9d5}.station h3{margin:18px 0 10px;font-family:var(--display);font-size:clamp(27px,3.5vw,52px);line-height:.95}.station:not(.station-primary) h3{font-size:28px}.station p{margin:0;color:var(--slate);font-size:11px}.station-primary p{color:#a8bcc0}.station-meta{display:flex;flex-wrap:wrap;gap:7px;margin-top:24px}.station-meta span{padding:5px 7px;border:1px solid #d1dcda;color:var(--slate);font-family:var(--mono);font-size:8px}.station-primary .station-meta span{color:#adc2c6;border-color:#526b73}.station-arrow{align-self:start;display:grid;width:38px;height:38px;place-items:center;border:1px solid currentColor;font-family:var(--mono)}.station.unavailable{pointer-events:none;opacity:.58}.status{position:absolute;top:22px;right:73px;padding:5px 7px;color:#08706c;background:var(--measure-soft);font-size:8px}.station-primary .status{color:#bafff9;background:#31545c}.status.idle{color:#6e777a;background:#e3e9e6}
.snapshot{margin-top:48px;border:1px solid var(--line);background:var(--panel)}.snapshot-head{display:flex;justify-content:space-between;gap:20px;align-items:end;padding:16px 18px;border-bottom:1px solid var(--line)}.snapshot-head h2{margin:0;font-family:var(--display);font-size:25px}.snapshot-head span{color:var(--slate);font-family:var(--mono);font-size:8px}.metrics{display:grid;grid-template-columns:repeat(6,1fr)}.metric{min-height:132px;padding:17px;border-right:1px solid var(--line)}.metric:last-child{border-right:0}.metric-label{display:block;color:var(--slate);font-size:8px}.metric strong{display:block;margin:19px 0 7px;font-family:var(--display);font-size:27px;line-height:1}.metric p{margin:0;color:var(--slate);font-size:9px}.metric.live strong{color:var(--measure)}.metric.warning strong{color:var(--amber)}.metric.fault strong{color:var(--fault)}
.curriculum{margin-top:48px}.phases{display:grid;grid-template-columns:repeat(5,1fr);border:1px solid var(--line);background:var(--panel)}.phase{position:relative;min-height:205px;padding:19px;border-right:1px solid var(--line)}.phase:last-child{border-right:0}.phase:not(:last-child):after{content:"→";position:absolute;z-index:1;top:23px;right:-9px;display:grid;width:18px;height:18px;place-items:center;color:#fff;background:var(--measure);font-family:var(--mono);font-size:8px}.phase-code{color:var(--measure);font-size:9px}.phase h3{margin:20px 0 9px;font-family:var(--display);font-size:20px}.phase p{margin:0;color:var(--slate);font-size:10px}.phase a{position:absolute;right:18px;bottom:16px;left:18px;padding-top:10px;border-top:1px solid var(--grid);color:#08706c;font-family:var(--mono);font-size:8px;text-decoration:none}.phase a:hover{color:var(--ink)}
.scope{display:grid;grid-template-columns:1fr 1fr;margin-top:48px;border:1px solid var(--line);background:var(--panel)}.scope>div{padding:20px}.scope>div+div{border-left:1px solid var(--line)}.scope h2{margin:0 0 10px;font-family:var(--display);font-size:23px}.scope p{margin:0;color:var(--slate);font-size:10px}.scope ul{margin:14px 0 0;padding-left:18px;color:var(--slate);font-size:10px}.scope li+li{margin-top:7px}.footer{padding:24px clamp(16px,4vw,56px);color:#9bb0b5;background:var(--blueprint);font-size:9px}.footer-inner{display:flex;justify-content:space-between;gap:20px;width:min(1368px,100%);margin:auto}.footer a{color:#78d7d2}
@media(max-width:1050px){.hero{grid-template-columns:1fr}.schematic{min-height:520px;border-left:0;border-top:1px solid #49616a}.metrics{grid-template-columns:repeat(3,1fr)}.metric:nth-child(3){border-right:0}.metric:nth-child(-n+3){border-bottom:1px solid var(--line)}.phases{grid-template-columns:repeat(3,1fr)}.phase:nth-child(3){border-right:0}.phase:nth-child(3):after{display:none}.phase:nth-child(-n+3){border-bottom:1px solid var(--line)}}
@media(max-width:720px){.run-id{display:none}.hero-copy{padding:50px 24px}.schematic{min-height:420px;padding:18px 12px}.schematic-readout{grid-template-columns:1fr}.system-rail{display:flex;overflow-x:auto}.rail-node{flex:0 0 130px}.main{padding-top:30px}.section-head{display:block}.section-head p{margin-top:8px}.stations{grid-template-columns:1fr;grid-template-rows:auto}.station-primary{grid-row:auto;min-height:320px}.station{min-height:220px}.metrics{grid-template-columns:1fr 1fr}.metric,.metric:nth-child(3){border-right:1px solid var(--line);border-bottom:1px solid var(--line)}.metric:nth-child(even){border-right:0}.metric:nth-last-child(-n+2){border-bottom:0}.phases{grid-template-columns:1fr}.phase{min-height:160px;border-right:0;border-bottom:1px solid var(--line)!important}.phase:after{display:none!important}.scope{grid-template-columns:1fr}.scope>div+div{border-left:0;border-top:1px solid var(--line)}.footer-inner{display:block}}
@media(prefers-reduced-motion:reduce){*{scroll-behavior:auto!important;animation:none!important;transition:none!important}.scan{animation:none}}
"""


_JS = r"""
(() => {
  const readout = document.getElementById('schematic-readout');
  const title = readout.querySelector('strong');
  const copy = readout.querySelector('span:last-child');
  document.querySelectorAll('.system-node').forEach((node) => {
    const show = () => {
      title.textContent = node.dataset.title;
      copy.textContent = node.dataset.copy;
    };
    node.addEventListener('mouseenter', show);
    node.addEventListener('focus', show);
  });
})();
"""


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
