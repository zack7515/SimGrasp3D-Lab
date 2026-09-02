"""建立可調參數的機械手、相機、軟管與障礙物學習工作台。"""

from __future__ import annotations

import json
from dataclasses import asdict
from html import escape
from pathlib import Path

import numpy as np
from plotly.offline import get_plotlyjs

from simgrasp3d.models.motion import HoseMotionSpec
from simgrasp3d.models.specs import SceneSpec
from simgrasp3d.models.system_design import SystemDesignLabResult
from simgrasp3d.robot.kinematics import forward_kinematics


_CSS = r"""
:root{--blueprint:#14272d;--blueprint-2:#203a42;--ink:#17262b;--slate:#62747a;--paper:#f3f6f3;--panel:#fbfcfa;--line:#c8d5d1;--grid:#dce6e2;--measure:#058b91;--measure-soft:#d9eeee;--amber:#d68d22;--amber-soft:#f7e8ca;--fault:#c64d3f;--fault-soft:#f5ddda;--white:#fff;--display:"Bahnschrift SemiCondensed","DIN Alternate","Arial Narrow",sans-serif;--body:Aptos,"Noto Sans TC","PingFang TC","Microsoft JhengHei",sans-serif;--mono:"JetBrains Mono","IBM Plex Mono",Consolas,monospace}
*{box-sizing:border-box}html{scroll-behavior:smooth}body{margin:0;color:var(--ink);background:var(--paper);font-family:var(--body);line-height:1.5}button,input{font:inherit}button{cursor:pointer}a{color:inherit}.skip{position:fixed;z-index:100;top:8px;left:8px;padding:8px 12px;background:#fff;transform:translateY(-180%)}.skip:focus{transform:none}:focus-visible{outline:3px solid var(--amber);outline-offset:3px}
.topbar{position:sticky;z-index:50;top:0;display:flex;justify-content:space-between;align-items:center;min-height:48px;padding:0 24px;color:#e9f4f2;background:rgba(20,39,45,.97);border-bottom:1px solid #47616a;backdrop-filter:blur(10px)}.brand,.mode,.eyebrow,.step-code,.control-unit,.gate-layer,.gate-state,.metric-key,.limit,.method-tag{font-family:var(--mono);text-transform:uppercase;letter-spacing:.09em}.brand{color:inherit;font-size:10px;text-decoration:none}.mode{color:#91a9af;font-size:9px}
.workflow{display:grid;grid-template-columns:repeat(5,1fr);color:#dceae7;background:var(--blueprint-2);border-bottom:1px solid #516971}.workflow a{position:relative;display:grid;grid-template-columns:38px 1fr;align-items:center;min-height:60px;padding:8px 18px;border-right:1px solid #47616a;text-decoration:none}.workflow a:last-child{border-right:0}.workflow a:hover{background:#294851}.step-code{color:#79d4d2;font-size:9px}.workflow strong{font-size:11px}.workflow span:last-child{display:block;color:#829aa1;font-size:9px}
.hero{position:relative;overflow:hidden;display:grid;grid-template-columns:minmax(0,1.25fr) minmax(300px,.75fr);min-height:310px;color:#f3faf8;background:var(--blueprint)}.hero-copy{align-self:end;padding:46px clamp(24px,5vw,78px)}.eyebrow{margin:0 0 14px;color:#7cd5d2;font-size:10px}.hero h1{max-width:900px;margin:0;font-family:var(--display);font-size:clamp(46px,7vw,94px);font-weight:700;letter-spacing:-.035em;line-height:.84}.hero-copy>p:last-child{max-width:780px;margin:24px 0 0;color:#afc1c4;font-size:14px}.envelopes{position:relative;overflow:hidden;display:grid;place-items:center;border-left:1px solid #405b63;background:#193139}.envelope-ring{position:absolute;border:1px solid rgba(113,215,211,.52);border-radius:50%}.ring-a{width:330px;height:330px}.ring-b{width:240px;height:240px;border-style:dashed}.ring-c{width:110px;height:110px;border-color:var(--amber)}.envelope-axis{position:absolute;width:82%;height:1px;background:#49636b}.envelope-axis.vertical{width:1px;height:82%}.envelope-caption{position:absolute;right:20px;bottom:18px;color:#78949b;font-family:var(--mono);font-size:8px;letter-spacing:.1em}.boundary{padding:12px clamp(24px,5vw,78px);color:#65430b;background:var(--amber-soft);border-bottom:1px solid #e0c184;font-size:11px}
.workspace{display:grid;grid-template-columns:minmax(0,1fr) 390px;gap:16px;width:min(1680px,100%);margin:0 auto;padding:18px}.canvas-card,.controls,.section{background:var(--panel);border:1px solid var(--line);box-shadow:0 10px 26px rgba(20,39,45,.06)}.card-head{display:flex;justify-content:space-between;align-items:flex-end;gap:20px;padding:15px 17px;border-bottom:1px solid var(--line)}.card-head h2,.section-head h2{margin:0;font-family:var(--display);font-size:23px}.card-head p,.section-head p{margin:3px 0 0;color:var(--slate);font-size:11px}.method-tag{flex:0 0 auto;padding:6px 8px;color:#096d71;background:var(--measure-soft);font-size:8px}.viewport{height:660px;min-height:500px}.viewport>div{width:100%!important;height:100%!important}.legend-strip{display:grid;grid-template-columns:repeat(4,1fr);border-top:1px solid var(--line)}.legend-strip div{padding:10px 12px;color:var(--slate);font-size:9px;border-right:1px solid var(--line)}.legend-strip div:last-child{border-right:0}.legend-strip b{display:block;color:var(--ink);font-family:var(--mono);font-size:8px;letter-spacing:.06em}
.controls{position:sticky;top:66px;align-self:start;max-height:calc(100vh - 84px);overflow:auto}.control-intro{padding:15px 16px;color:#d9e8e5;background:var(--blueprint-2)}.control-intro strong{display:block;font-family:var(--display);font-size:22px}.control-intro span{color:#9bb1b6;font-size:10px}.presets{display:grid;grid-template-columns:1fr 1fr;gap:6px;padding:10px;border-bottom:1px solid var(--line)}.preset{min-height:46px;padding:8px;text-align:left;color:var(--ink);background:#fff;border:1px solid var(--line);font-size:9px}.preset:hover,.preset.active{color:#fff;background:var(--measure);border-color:var(--measure)}.control-group{border-bottom:1px solid var(--line)}.control-group summary{padding:12px 14px;color:var(--blueprint);background:#eef3f0;font-family:var(--mono);font-size:9px;letter-spacing:.08em;cursor:pointer}.control-list{padding:3px 14px 12px}.control{padding:11px 0;border-bottom:1px solid #e3eae7}.control:last-child{border-bottom:0}.control-title{display:flex;justify-content:space-between;gap:12px;font-size:11px}.control-output{color:var(--measure);font-family:var(--mono);font-weight:700}.control input{width:100%;margin:8px 0 2px;accent-color:var(--measure)}.control-scale{display:flex;justify-content:space-between;color:#839399;font-family:var(--mono);font-size:8px}.control p{margin:6px 0 0;color:var(--slate);font-size:9px}.control-actions{position:sticky;bottom:0;display:grid;grid-template-columns:1fr 1fr;gap:7px;padding:10px;background:rgba(251,252,250,.96);border-top:1px solid var(--line);backdrop-filter:blur(8px)}.action{min-height:38px;padding:8px;border:1px solid var(--blueprint);color:#fff;background:var(--blueprint);font-family:var(--mono);font-size:8px;letter-spacing:.05em}.action.secondary{color:var(--blueprint);background:#fff}.action:hover{border-color:var(--measure)}
.analysis{width:min(1680px,100%);margin:0 auto;padding:0 18px 50px}.section{margin-top:16px}.section-head{display:flex;justify-content:space-between;align-items:end;gap:20px;padding:14px 16px;border-bottom:1px solid var(--line)}.score{font-family:var(--display);font-size:30px}.score span{color:var(--slate);font-size:15px}.gates{display:grid;grid-template-columns:repeat(3,1fr)}.gate{position:relative;min-height:175px;padding:16px;border-right:1px solid var(--line);border-bottom:1px solid var(--line)}.gate:before{content:"";position:absolute;top:0;right:0;left:0;height:4px;background:var(--measure)}.gate.fail:before{background:var(--fault)}.gate-layer{color:var(--slate);font-size:8px}.gate-state{float:right;color:#08746d;font-size:8px}.gate.fail .gate-state{color:var(--fault)}.gate-value{display:block;margin:20px 0 7px;font-family:var(--display);font-size:31px;line-height:1}.gate h3{margin:0;font-size:12px}.gate p{margin:9px 0 0;color:var(--slate);font-size:9px}.gate-action{padding-top:8px;border-top:1px solid var(--grid);color:#5a490e!important}.limit{margin-top:7px;color:var(--slate);font-size:8px}
.compare-wrap{overflow-x:auto}.compare{width:100%;border-collapse:collapse;font-size:10px}.compare th,.compare td{padding:10px 12px;border:1px solid var(--line);text-align:left}.compare th{color:var(--slate);background:#eef3f0;font-family:var(--mono);font-size:8px;letter-spacing:.06em}.delta-bad{color:var(--fault)}.delta-good{color:#08746d}.architecture{display:grid;grid-template-columns:repeat(6,1fr)}.layer{position:relative;min-height:190px;padding:16px;border-right:1px solid var(--line)}.layer:last-child{border-right:0}.layer:not(:last-child):after{content:"→";position:absolute;z-index:2;right:-9px;top:22px;display:grid;width:18px;height:18px;place-items:center;color:#fff;background:var(--measure);font-family:var(--mono);font-size:9px}.layer-code{color:var(--measure);font-family:var(--mono);font-size:9px}.layer h3{margin:13px 0 8px;font-family:var(--display);font-size:18px}.layer p{margin:0;color:var(--slate);font-size:9px}.layer dl{margin:13px 0 0;font-size:9px}.layer dt{color:var(--slate);font-family:var(--mono);font-size:8px}.layer dd{margin:2px 0 9px}.lesson-grid{display:grid;grid-template-columns:1fr 1fr}.lesson{padding:18px}.lesson+ .lesson{border-left:1px solid var(--line)}.lesson h3{margin:0 0 10px;font-family:var(--display);font-size:19px}.lesson ol{margin:0;padding-left:19px;color:var(--slate);font-size:10px}.lesson li+li{margin-top:8px}.log-empty{padding:22px;color:var(--slate);font-size:11px}.log-table{display:none;width:100%;border-collapse:collapse;font-size:9px}.log-table th,.log-table td{padding:9px 10px;border:1px solid var(--line);text-align:left}.log-table th{font-family:var(--mono);font-size:8px}.footer{padding:24px 18px;color:#9cb0b5;background:var(--blueprint);font-size:10px}.footer-inner{display:flex;justify-content:space-between;gap:20px;width:min(1644px,100%);margin:auto}.footer a{color:#78d2d0}
@media(max-width:1180px){.workspace{grid-template-columns:1fr}.controls{position:static;max-height:none}.gates{grid-template-columns:repeat(2,1fr)}.architecture{grid-template-columns:repeat(3,1fr)}.layer:nth-child(3){border-right:0}.layer:nth-child(3):after{display:none}}
@media(max-width:760px){.topbar{padding:0 12px}.mode{display:none}.workflow{display:flex;overflow-x:auto}.workflow a{flex:0 0 155px}.hero{grid-template-columns:1fr}.envelopes{display:none}.workspace,.analysis{padding-left:8px;padding-right:8px}.viewport{height:560px}.legend-strip{grid-template-columns:1fr 1fr}.gates{grid-template-columns:1fr}.gate{min-height:0}.architecture{grid-template-columns:1fr}.layer{min-height:0;border-right:0;border-bottom:1px solid var(--line)}.layer:after{display:none!important}.lesson-grid{grid-template-columns:1fr}.lesson+ .lesson{border-left:0;border-top:1px solid var(--line)}.footer-inner{display:block}}
@media(prefers-reduced-motion:reduce){*{scroll-behavior:auto!important;animation:none!important;transition:none!important}}
"""


def _payload(result: SystemDesignLabResult, scene: SceneSpec, motion: HoseMotionSpec) -> dict:
    baseline = result.baseline
    joint_positions, _, tool_frame = forward_kinematics(scene.robot)
    nominal_reach = sum(float(np.linalg.norm(link.translation)) for link in scene.robot.links)
    nominal_reach += float(np.linalg.norm(scene.robot.gripper.tcp_offset))
    return {
        "rawSpec": {
            "name": result.spec.name,
            "seed": result.spec.seed,
            "scenario_summary": result.spec.scenario_summary,
            "parameters": [asdict(item) for item in result.spec.parameters],
            "thresholds": result.spec.thresholds,
            "presets": [asdict(item) for item in result.spec.presets],
        },
        "parameters": [asdict(item) for item in result.spec.parameters],
        "presets": [asdict(item) for item in result.spec.presets],
        "thresholds": result.spec.thresholds,
        "baselineGates": [asdict(item) for item in baseline.gates],
        "baselineValues": baseline.values,
        "geometry": {
            "tableSize": list(scene.table.size),
            "tableCenter": list(scene.table.pose.xyz),
            "basePosition": list(scene.robot.base_pose.xyz),
            "shoulderPosition": [
                scene.robot.base_pose.xyz[0],
                scene.robot.base_pose.xyz[1],
                scene.robot.base_pose.xyz[2] + scene.robot.base_size[2],
            ],
            "robotJointPositions": joint_positions.tolist(),
            "robotToolPosition": tool_frame[:3, 3].tolist(),
            "robotLinkNames": [item.name for item in scene.robot.links],
            "nominalReach": nominal_reach,
            "cameraX": scene.camera.position[0],
            "cameraLookAt": list(scene.camera.look_at),
            "cameraAspect": scene.camera.aspect_ratio,
            "cameraNear": scene.camera.near,
            "cameraFar": scene.camera.far,
            "noise": asdict(scene.camera.noise),
            "hosePoints": baseline.hose_points.tolist(),
            "goalPoint": list(motion.target_position),
            "startPoint": list(motion.keyframes[0].tcp_position),
            "tableTopZ": motion.table_top_z,
            "toolEnvelopeRadius": motion.waypoint_planner.tool_envelope_radius_m,
            "detourStep": motion.waypoint_planner.detour_step_m,
            "maximumDetour": motion.waypoint_planner.maximum_detour_m,
            "obstacles": [
                {"name": item.name, "start": list(item.start), "end": list(item.end), "radius": item.radius}
                for item in motion.obstacles
            ],
            "minimumBendRadius": baseline.metrics["minimum_bend_radius_m"],
        },
    }


def write_system_design_lab(
    result: SystemDesignLabResult,
    scene: SceneSpec,
    motion: HoseMotionSpec,
    output_path: str | Path,
) -> Path:
    """輸出可離線調參、記錄實驗並下載 JSON 的自包含學習頁。"""

    destination = Path(output_path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    data = json.dumps(_payload(result, scene, motion), ensure_ascii=False).replace("</", "<\\/")
    title = "系統設計實驗室｜SimGrasp3D"
    document = f'''<!doctype html>
<html lang="zh-Hant"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>{escape(title)}</title><style>{_CSS}</style><script>{get_plotlyjs()}</script></head>
<body><a class="skip" href="#lab-main">跳到系統設計工作台</a>
<header class="topbar"><a class="brand" href="index.html">← SIMGRASP3D / SYSTEM DESIGN LAB</a><span class="mode">FAST GEOMETRY ESTIMATOR · SIMULATION ONLY</span></header>
<nav class="workflow" aria-label="系統設計順序">
  <a href="#scenario"><span class="step-code">01</span><strong>定義任務<span>Pick / Place / Constraint</span></strong></a>
  <a href="#workspace"><span class="step-code">02</span><strong>配置硬體<span>Arm / Camera / Gripper</span></strong></a>
  <a href="#gates"><span class="step-code">03</span><strong>建立模型<span>Frames / Error / Envelope</span></strong></a>
  <a href="#architecture"><span class="step-code">04</span><strong>規劃動作<span>Grasp / IK / Collision</span></strong></a>
  <a href="#experiment"><span class="step-code">05</span><strong>做實驗<span>Record / Compare / Refine</span></strong></a>
</nav>
<section class="hero" id="scenario"><div class="hero-copy"><p class="eyebrow">桌上型軟管取放 / 教學情境</p><h1>先畫出安全包絡，<br>再談抓取。</h1><p>{escape(result.spec.scenario_summary)} 此頁不是展示答案，而是讓你找出哪個子系統先失敗，以及下一個應該修改的參數。</p></div>
<div class="envelopes" aria-hidden="true"><span class="envelope-ring ring-a"></span><span class="envelope-ring ring-b"></span><span class="envelope-ring ring-c"></span><span class="envelope-axis"></span><span class="envelope-axis vertical"></span><span class="envelope-caption">CAMERA FRUSTUM × REACH × COLLISION</span></div></section>
<div class="boundary">學習邊界：即時頁面執行視錐、工作空間、管徑、曲率與 capsule 淨空估算；六軸姿態 IK、逐點 RGB-D、軟管接觸力與控制安全仍須重新執行 Python 完整管線。</div>
<main id="lab-main"><div class="workspace" id="workspace">
  <section class="canvas-card"><header class="card-head"><div><h2>情境幾何與三種包絡</h2><p>拖曳旋轉，觀察青色相機視錐、虛線工作空間與琥珀色路徑如何互相限制。</p></div><span class="method-tag">LIVE ESTIMATE</span></header><div id="design-view" class="viewport" aria-label="可調參數 3D 情境"></div>
  <div class="legend-strip"><div><b>CAMERA</b>視錐與遮蔽</div><div><b>ROBOT</b>球形早期篩選</div><div><b>HOSE</b>管徑與彎曲</div><div><b>PLANNER</b>工具安全包絡</div></div></section>
  <aside class="controls" aria-label="設計參數"><div class="control-intro"><strong>調參控制台</strong><span>一次先改一個變數，記錄假設，再比較安全閘門。</span></div><div id="preset-list" class="presets"></div><div id="control-groups"></div>
  <div class="control-actions"><button class="action secondary" id="reset-design">還原基準</button><button class="action" id="record-design">記錄本次實驗</button><button class="action secondary" id="download-config">下載參數 JSON</button><button class="action secondary" id="export-log">匯出實驗 CSV</button></div></aside>
</div>
<div class="analysis"><section class="section" id="gates"><header class="section-head"><div><h2>六道設計閘門</h2><p>STOP 代表這組幾何條件不應進入下一層，不代表其他 PASS 已完成真實驗證。</p></div><div class="score" id="gate-score">—</div></header><div class="gates" id="gate-grid"></div></section>
<section class="section"><header class="section-head"><div><h2>基準與目前設計比較</h2><p>正負變化是否改善，取決於每個指標的方向，不應只追求數字變大。</p></div></header><div class="compare-wrap"><table class="compare"><thead><tr><th>閘門</th><th>基準</th><th>目前</th><th>判定</th><th>下一步</th></tr></thead><tbody id="compare-body"></tbody></table></div></section>
<section class="section" id="architecture"><header class="section-head"><div><h2>這套系統真正要設計的資料流</h2><p>每一層都要有輸入、輸出與可量測失敗條件，不能只看最後動畫。</p></div></header><div class="architecture">
  <article class="layer"><span class="layer-code">L0 / TASK</span><h3>任務與約束</h3><p>先定義夾哪裡、放哪裡、不可碰什麼。</p><dl><dt>INPUT</dt><dd>軟管規格、目標、禁區</dd><dt>OUTPUT</dt><dd>成功條件與門檻</dd></dl></article>
  <article class="layer"><span class="layer-code">L1 / WORLD</span><h3>尺寸與座標</h3><p>URDF、相機外參、桌面與障礙幾何必須共用 meter。</p><dl><dt>MEASURE</dt><dd>TCP、base、camera→world</dd><dt>FAIL</dt><dd>frame 或單位混用</dd></dl></article>
  <article class="layer"><span class="layer-code">L2 / SENSE</span><h3>RGB-D 感知</h3><p>投影、遮蔽、深度誤差後才是機器真正看到的點雲。</p><dl><dt>OUTPUT</dt><dd>桌面、軟管、障礙點雲</dd><dt>FAIL</dt><dd>不可見或誤差超容差</dd></dl></article>
  <article class="layer"><span class="layer-code">L3 / GRASP</span><h3>夾取設計</h3><p>選中心線位置、接近方向、開口與允許接觸力。</p><dl><dt>OUTPUT</dt><dd>pregrasp / grasp pose</dd><dt>FAIL</dt><dd>管徑、曲率或材質不符</dd></dl></article>
  <article class="layer"><span class="layer-code">L4 / PLAN</span><h3>IK 與避障</h3><p>先用包絡篩選，再跑關節限制、連續碰撞與軟管 swept volume。</p><dl><dt>OUTPUT</dt><dd>有時間參數的安全軌跡</dd><dt>FAIL</dt><dd>不可達、碰撞、拉扯</dd></dl></article>
  <article class="layer"><span class="layer-code">L5 / VERIFY</span><h3>物理與安全</h3><p>材料校正、接觸力、失敗注入與 fail-closed 才能支持下一階段。</p><dl><dt>OUTPUT</dt><dd>證據、回歸與停止原因</dd><dt>FAIL</dt><dd>不可觀測或超門檻</dd></dl></article>
</div></section>
<section class="section"><div class="lesson-grid"><article class="lesson"><h3>建議操作順序</h3><ol><li>按「基準設計」，先解釋每一個閘門的物理意義。</li><li>選一個失敗 preset，只修改對應子系統的一個參數。</li><li>記錄假設、目前值、STOP 數量與路徑長度。</li><li>下載 JSON，使用 Python 管線重新產生 RGB-D、IK、碰撞與物理結果。</li><li>比較 fast estimator 與完整模擬不一致之處，修正模型。</li></ol></article><article class="lesson"><h3>參數不是越大越好</h3><ol><li>提高相機可視範圍可能降低空間解析度。</li><li>增加手臂長度會放大慣量、撓曲與碰撞包絡。</li><li>提高安全距離可能讓狹窄空間完全無解。</li><li>增加抬升高度可能避障，卻讓目標超出工作空間。</li><li>降低軟管彎曲限制不是調參捷徑，必須來自材料規格或校正。</li></ol></article></div></section>
<section class="section" id="experiment"><header class="section-head"><div><h2>實驗紀錄</h2><p>這份紀錄只存在目前瀏覽器頁面；匯出 CSV 後才能進行版本比較。</p></div></header><div class="log-empty" id="log-empty">尚未記錄。先改一個參數，說出預期，再按「記錄本次實驗」。</div><div class="compare-wrap"><table class="log-table" id="log-table"><thead><tr><th>#</th><th>時間</th><th>PASS</th><th>STOP</th><th>路徑長度</th><th>waypoint</th><th>主要失敗</th></tr></thead><tbody id="log-body"></tbody></table></div></section>
</div></main>
<footer class="footer"><div class="footer-inner"><span>SimGrasp3D System Design Lab · simulation-only · thresholds uncalibrated</span><span><a href="index.html">專案主頁</a> · <a href="simulation_report.html">整合驗證報告</a> · <a href="hospital/index.html">醫院案例</a></span></div></footer>
<script id="lab-data" type="application/json">{data}</script>
<script>
(() => {{
  'use strict';
  const DATA = JSON.parse(document.getElementById('lab-data').textContent);
  const G = DATA.geometry, T = DATA.thresholds;
  const values = Object.assign({{}}, DATA.baselineValues);
  const baselineValues = Object.assign({{}}, DATA.baselineValues);
  const experimentLog = [];
  const add=(a,b)=>a.map((v,i)=>v+b[i]), sub=(a,b)=>a.map((v,i)=>v-b[i]), mul=(a,s)=>a.map(v=>v*s);
  const dot=(a,b)=>a.reduce((s,v,i)=>s+v*b[i],0), norm=a=>Math.sqrt(dot(a,a));
  const unit=a=>{{const n=norm(a);return n<1e-12?[0,0,0]:mul(a,1/n)}};
  const cross=(a,b)=>[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];
  function segmentDistance(p1,q1,p2,q2) {{
    const d1=sub(q1,p1), d2=sub(q2,p2), r=sub(p1,p2), a=dot(d1,d1), e=dot(d2,d2), eps=1e-12;
    let s=0,t=0;
    if(a<=eps&&e<=eps)return norm(r);
    if(a<=eps) t=Math.max(0,Math.min(1,dot(d2,r)/e));
    else {{
      const c=dot(d1,r);
      if(e<=eps)s=Math.max(0,Math.min(1,-c/a));
      else {{const b=dot(d1,d2),den=a*e-b*b;s=den!==0?Math.max(0,Math.min(1,(b*dot(d2,r)-c*e)/den)):0;t=(b*s+dot(d2,r))/e;if(t<0){{t=0;s=Math.max(0,Math.min(1,-c/a))}}else if(t>1){{t=1;s=Math.max(0,Math.min(1,(b-c)/a))}}}}
    }}
    return norm(sub(add(p1,mul(d1,s)),add(p2,mul(d2,t))));
  }}
  const toolRadius=()=>Math.max(G.toolEnvelopeRadius,values.gripper_command_m/2+0.025);
  const obstacles=()=>G.obstacles.map(o=>({{...o,radius:o.radius*values.obstacle_radius_scale}}));
  function clearance(a,b) {{return Math.min(...obstacles().map(o=>segmentDistance(a,b,o.start,o.end)-toolRadius()-o.radius))}}
  function candidateOffsets() {{const out=[];for(let d=G.detourStep;d<=G.maximumDetour+1e-10;d+=G.detourStep)out.push([0,0,d],[0,d,d*.5],[0,-d,d*.5],[d,0,d*.5],[-d,0,d*.5]);return out}}
  function planPath(grasp,goal) {{
    const lift=values.lift_height_m, approach=Math.max(.10,Math.min(.18,lift*.55));
    const raw=[G.startPoint,add(grasp,[0,0,approach]),grasp,grasp,add(grasp,[0,0,lift]),add(goal,[0,0,lift]),goal,goal,add(goal,[0,0,lift])];
    const planned=[raw[0]];let inserted=0,unresolved=0;
    for(let i=1;i<raw.length;i++){{const a=planned[planned.length-1],b=raw[i];if(norm(sub(a,b))<1e-10){{planned.push(b);continue}}if(clearance(a,b)>=values.safety_margin_m){{planned.push(b);continue}}const mid=mul(add(a,b),.5), candidates=[];candidateOffsets().forEach(off=>{{const p=add(mid,off),c=Math.min(clearance(a,p),clearance(p,b));if(c>=values.safety_margin_m)candidates.push([norm(sub(p,a))+norm(sub(b,p)),p])}});if(!candidates.length)unresolved++;else{{candidates.sort((x,y)=>x[0]-y[0]);planned.push(candidates[0][1]);inserted++}}planned.push(b)}}
    return {{path:planned,inserted,unresolved}};
  }}
  function cameraCoverage(points,pos,look) {{
    const f=unit(sub(look,pos)),r0=cross(f,[0,0,1]),r=norm(r0)<1e-9?[1,0,0]:unit(r0),u=unit(cross(r,f)),tan=Math.tan(values.camera_fov_deg*Math.PI/360);let frustum=0,visible=0;
    points.forEach(p=>{{const rel=sub(p,pos),z=dot(rel,f),inside=z>=G.cameraNear&&z<=G.cameraFar&&Math.abs(dot(rel,u))<=z*tan&&Math.abs(dot(rel,r))<=z*tan*G.cameraAspect;if(!inside)return;frustum++;let blocked=false;for(const o of obstacles()){{if(segmentDistance(pos,p,o.start,o.end)<=o.radius){{blocked=true;break}}}}if(!blocked)visible++}});
    return {{frustum:frustum/points.length,visible:visible/points.length}};
  }}
  const fmt=(v,u)=>u==='ratio'?`${{(v*100).toFixed(1)}}%`:u==='m'?`${{(v*1000).toFixed(1)}} mm`:u==='deg'?`${{v.toFixed(1)}}°`:u==='x'?`${{v.toFixed(2)}}×`:v.toFixed(3);
  function evaluate() {{
    const index=Math.round(values.grasp_fraction*(G.hosePoints.length-1)),grasp=[...G.hosePoints[index]],goal=[...G.goalPoint];grasp[2]=Math.max(grasp[2],G.tableTopZ+values.hose_radius_m);goal[2]=Math.max(goal[2],G.tableTopZ+values.hose_radius_m);
    const cam=[G.cameraX,values.camera_lateral_m,values.camera_height_m],coverage=cameraCoverage([...G.hosePoints,goal],cam,G.cameraLookAt),distance=norm(sub(grasp,cam)),n=G.noise;
    const axial=n.axial_noise_std_base_m+n.axial_noise_std_per_m2*distance*distance,rot=distance*n.extrinsic_rotation_std_deg*Math.PI/180,unc=3*Math.sqrt(axial*axial+n.extrinsic_translation_std_m**2+rot*rot+(n.depth_quantization_m/Math.sqrt(12))**2)*values.depth_noise_scale;
    const plan=planPath(grasp,goal),shoulder=G.shoulderPosition,maxReach=G.nominalReach*values.arm_reach_scale,maxRequest=Math.max(...plan.path.map(p=>norm(sub(p,shoulder)))),reserve=maxReach-maxRequest;
    const gripError=Math.abs(values.gripper_command_m-2*values.hose_radius_m);let minClear=Infinity;for(let i=0;i<plan.path.length-1;i++)if(norm(sub(plan.path[i],plan.path[i+1]))>1e-10)minClear=Math.min(minClear,clearance(plan.path[i],plan.path[i+1]));
    const meta=DATA.baselineGates.reduce((o,g)=>(o[g.key]=g,o),{{}}), gates=[
      {{...meta.camera_coverage,value:coverage.visible,limit:T.minimum_visibility_ratio,passed:coverage.visible>=T.minimum_visibility_ratio}},
      {{...meta.depth_uncertainty,value:unc,limit:T.maximum_depth_uncertainty_m,passed:unc<=T.maximum_depth_uncertainty_m}},
      {{...meta.reach_reserve,value:reserve,limit:T.minimum_reach_reserve_m,passed:reserve>=T.minimum_reach_reserve_m}},
      {{...meta.gripper_match,value:gripError,limit:T.maximum_gripper_diameter_error_m,passed:gripError<=T.maximum_gripper_diameter_error_m}},
      {{...meta.bend_radius,value:G.minimumBendRadius,limit:values.hose_min_bend_radius_m,passed:G.minimumBendRadius>=values.hose_min_bend_radius_m}},
      {{...meta.path_clearance,value:minClear,limit:values.safety_margin_m,passed:minClear>=values.safety_margin_m&&plan.unresolved===0}}
    ];
    const length=plan.path.slice(1).reduce((s,p,i)=>s+norm(sub(p,plan.path[i])),0);
    return {{grasp,goal,cam,coverage,unc,reserve,maxReach,plan,minClear,length,gates}};
  }}
  function frustumLines(cam) {{
    const f=unit(sub(G.cameraLookAt,cam)),r=unit(cross(f,[0,0,1])),u=unit(cross(r,f)),d=Math.min(G.cameraFar,1.55),hh=Math.tan(values.camera_fov_deg*Math.PI/360)*d,hw=hh*G.cameraAspect,c=add(cam,mul(f,d));
    const corners=[add(add(c,mul(r,-hw)),mul(u,-hh)),add(add(c,mul(r,hw)),mul(u,-hh)),add(add(c,mul(r,hw)),mul(u,hh)),add(add(c,mul(r,-hw)),mul(u,hh))],x=[],y=[],z=[];
    const seg=(a,b)=>{{x.push(a[0],b[0],null);y.push(a[1],b[1],null);z.push(a[2],b[2],null)}};corners.forEach((p,i)=>{{seg(cam,p);seg(p,corners[(i+1)%4])}});return {{x,y,z}}
  }}
  function ring(center,radius,plane) {{const pts=[];for(let i=0;i<=72;i++){{const a=2*Math.PI*i/72;pts.push(plane==='xy'?[center[0]+radius*Math.cos(a),center[1]+radius*Math.sin(a),center[2]]:[center[0]+radius*Math.cos(a),center[1],center[2]+radius*Math.sin(a)])}}return pts}}
  function renderPlot(state) {{
    const table=G.tableCenter,half=G.tableSize.map(v=>v/2),verts=[];[[-1,-1,-1],[1,-1,-1],[1,1,-1],[-1,1,-1],[-1,-1,1],[1,-1,1],[1,1,1],[-1,1,1]].forEach(s=>verts.push(s.map((v,i)=>table[i]+v*half[i])));
    const traces=[{{type:'mesh3d',x:verts.map(p=>p[0]),y:verts.map(p=>p[1]),z:verts.map(p=>p[2]),i:[0,0,4,4,0,0,1,1,2,2,3,3],j:[1,2,5,6,1,5,2,6,3,7,0,4],k:[2,3,6,7,5,4,6,5,7,6,4,7],color:'#bccac3',opacity:.35,name:'工作桌',hoverinfo:'skip'}},
      {{type:'scatter3d',mode:'lines',x:G.hosePoints.map(p=>p[0]),y:G.hosePoints.map(p=>p[1]),z:G.hosePoints.map(p=>p[2]),line:{{color:'#06999c',width:Math.max(5,values.hose_radius_m*300)}},name:'軟管中心線'}},
      {{type:'scatter3d',mode:'lines+markers',x:state.plan.path.map(p=>p[0]),y:state.plan.path.map(p=>p[1]),z:state.plan.path.map(p=>p[2]),line:{{color:state.gates[5].passed?'#d68d22':'#c64d3f',width:6}},marker:{{size:3,color:'#d68d22'}},name:'規劃 TCP 路徑'}},
      {{type:'scatter3d',mode:'markers',x:[state.grasp[0]],y:[state.grasp[1]],z:[state.grasp[2]],marker:{{size:8,color:'#f0ad31',symbol:'diamond',line:{{color:'#fff',width:2}}}},name:'夾取點'}},
      {{type:'scatter3d',mode:'markers',x:[state.goal[0]],y:[state.goal[1]],z:[state.goal[2]],marker:{{size:9,color:'#7656a5',symbol:'circle-open',line:{{width:4}}}},name:'放置點'}},
      {{type:'scatter3d',mode:'markers+text',x:[state.cam[0]],y:[state.cam[1]],z:[state.cam[2]],text:['RGB-D'],textposition:'top center',marker:{{size:7,color:'#058b91'}},name:'相機'}},
    ];
    obstacles().forEach(o=>traces.push({{type:'scatter3d',mode:'lines',x:[o.start[0],o.end[0]],y:[o.start[1],o.end[1]],z:[o.start[2],o.end[2]],line:{{color:'#52656c',width:Math.max(9,o.radius*260)}},name:o.name,showlegend:false}}));
    const fr=frustumLines(state.cam);traces.push({{type:'scatter3d',mode:'lines',...fr,line:{{color:'#058b91',width:2,dash:'dot'}},name:'相機視錐',showlegend:false}});
    ['xy','xz'].forEach(plane=>{{const pts=ring(G.shoulderPosition,state.maxReach,plane);traces.push({{type:'scatter3d',mode:'lines',x:pts.map(p=>p[0]),y:pts.map(p=>p[1]),z:pts.map(p=>p[2]),line:{{color:'#7a8e94',width:2,dash:'dash'}},opacity:.55,name:'工作空間包絡',showlegend:false}})}});
    const scaledJoints=G.robotJointPositions.map(p=>add(G.shoulderPosition,mul(sub(p,G.shoulderPosition),values.arm_reach_scale))),scaledTool=add(G.shoulderPosition,mul(sub(G.robotToolPosition,G.shoulderPosition),values.arm_reach_scale));scaledJoints.push(scaledTool);
    traces.push({{type:'scatter3d',mode:'lines+markers',x:scaledJoints.map(p=>p[0]),y:scaledJoints.map(p=>p[1]),z:scaledJoints.map(p=>p[2]),line:{{color:'#233c44',width:10}},marker:{{size:5,color:'#d68d22',line:{{color:'#fff',width:1}}}},name:'六軸手臂結構'}});
    traces.push({{type:'scatter3d',mode:'lines+markers',x:[G.basePosition[0],G.shoulderPosition[0]],y:[G.basePosition[1],G.shoulderPosition[1]],z:[G.basePosition[2],G.shoulderPosition[2]],line:{{color:'#233c44',width:14}},marker:{{size:7,color:'#d68d22'}},name:'機械手基座',showlegend:false}});
    const halfGrip=values.gripper_command_m/2,gx=state.grasp[0],gy=state.grasp[1],gz=state.grasp[2]+.018;traces.push({{type:'scatter3d',mode:'lines',x:[gx-.07,gx,null,gx-.07,gx],y:[gy+halfGrip,gy+halfGrip,null,gy-halfGrip,gy-halfGrip],z:[gz,gz,gz,gz,gz],line:{{color:state.gates[3].passed?'#d68d22':'#c64d3f',width:8}},name:'閉爪開口示意'}});
    Plotly.react('design-view',traces,{{margin:{{l:0,r:0,t:0,b:0}},paper_bgcolor:'#fbfcfa',showlegend:true,legend:{{orientation:'h',x:0,y:1.02,font:{{size:9}}}},font:{{family:'Aptos, Noto Sans TC, sans-serif',color:'#17262b',size:10}},scene:{{xaxis:{{title:'X (m)',range:[-.9,.95],backgroundcolor:'#edf3f0',gridcolor:'#c8d5d1',showbackground:true}},yaxis:{{title:'Y (m)',range:[-.75,.75],backgroundcolor:'#edf3f0',gridcolor:'#c8d5d1',showbackground:true}},zaxis:{{title:'Z (m)',range:[0,1.65],backgroundcolor:'#edf3f0',gridcolor:'#c8d5d1',showbackground:true}},aspectmode:'manual',aspectratio:{{x:1.1,y:.9,z:1}},camera:{{eye:{{x:1.5,y:-1.6,z:1.0}}}},uirevision:'system-design'}},responsive:true}},{{displaylogo:false,responsive:true,scrollZoom:true}});
  }}
  function renderGates(state) {{
    const passed=state.gates.filter(g=>g.passed).length;document.getElementById('gate-score').innerHTML=`${{passed}} <span>/ ${{state.gates.length}} PASS</span>`;
    document.getElementById('gate-grid').innerHTML=state.gates.map(g=>`<article class="gate ${{g.passed?'':'fail'}}"><span class="gate-layer">${{g.layer}}</span><span class="gate-state">${{g.passed?'PASS':'STOP'}}</span><strong class="gate-value">${{fmt(g.value,g.unit)}}</strong><h3>${{g.label}}</h3><div class="limit">TEACHING GATE ${{g.relation}} ${{fmt(g.limit,g.unit)}} · 未校準</div><p>${{g.explanation}}</p><p class="gate-action">調整方向：${{g.action}}</p></article>`).join('');
    const baseline=DATA.baselineGates.reduce((o,g)=>(o[g.key]=g,o),{{}});document.getElementById('compare-body').innerHTML=state.gates.map(g=>{{const b=baseline[g.key],same=g.passed===b.passed;return `<tr><td>${{g.label}}</td><td>${{fmt(b.value,b.unit)}} / ${{b.passed?'PASS':'STOP'}}</td><td>${{fmt(g.value,g.unit)}}</td><td class="${{g.passed?'delta-good':'delta-bad'}}">${{g.passed?'PASS':'STOP'}}${{same?'':' / 狀態改變'}}</td><td>${{g.action}}</td></tr>`}}).join('');
  }}
  function update() {{document.querySelectorAll('.preset').forEach(x=>x.classList.remove('active'));const state=evaluate();renderPlot(state);renderGates(state);return state}}
  function displayParameter(p,value) {{return p.unit==='ratio'?`${{Math.round(value*100)}}%`:p.unit==='m'?`${{(value*1000).toFixed(0)}} mm`:p.unit==='deg'?`${{value.toFixed(0)}}°`:p.unit==='x'?`${{value.toFixed(2)}}×`:String(value)}}
  function buildControls() {{
    const groups={{}};DATA.parameters.forEach(p=>(groups[p.group]??=[]).push(p));const host=document.getElementById('control-groups');
    Object.entries(groups).forEach(([name,params],groupIndex)=>{{const details=document.createElement('details');details.className='control-group';details.open=groupIndex===0;const summary=document.createElement('summary');summary.textContent=`${{String(groupIndex+1).padStart(2,'0')}} / ${{name}}`;details.appendChild(summary);const list=document.createElement('div');list.className='control-list';params.forEach(p=>{{const row=document.createElement('div');row.className='control';row.innerHTML=`<label class="control-title" for="control-${{p.key}}"><span>${{p.label}}</span><output class="control-output" id="value-${{p.key}}"></output></label><input id="control-${{p.key}}" type="range" min="${{p.minimum}}" max="${{p.maximum}}" step="${{p.step}}" value="${{p.value}}"><div class="control-scale"><span>${{displayParameter(p,p.minimum)}}</span><span>${{displayParameter(p,p.maximum)}}</span></div><p>${{p.description}}</p>`;list.appendChild(row);const input=row.querySelector('input'),output=row.querySelector('output');const sync=()=>{{values[p.key]=Number(input.value);output.value=displayParameter(p,values[p.key]);update()}};input.addEventListener('input',sync);output.value=displayParameter(p,p.value)}});details.appendChild(list);host.appendChild(details)}});
    const presets=document.getElementById('preset-list');DATA.presets.forEach((p,i)=>{{const button=document.createElement('button');button.className='preset';button.textContent=p.name;button.title=p.description;button.addEventListener('click',()=>{{Object.assign(values,baselineValues,p.values);syncInputs();update();button.classList.add('active')}});presets.appendChild(button);if(i===0)button.classList.add('active')}});
  }}
  function syncInputs() {{DATA.parameters.forEach(p=>{{const input=document.getElementById(`control-${{p.key}}`),output=document.getElementById(`value-${{p.key}}`);input.value=values[p.key];output.value=displayParameter(p,values[p.key])}})}}
  document.getElementById('reset-design').addEventListener('click',()=>{{Object.assign(values,baselineValues);syncInputs();update();document.querySelector('.preset')?.classList.add('active')}});
  document.getElementById('record-design').addEventListener('click',()=>{{const s=evaluate(),failed=s.gates.filter(g=>!g.passed);experimentLog.push({{time:new Date().toLocaleTimeString('zh-TW'),pass:s.gates.length-failed.length,stop:failed.length,length:s.length,waypoint:s.plan.inserted,fail:failed.map(g=>g.label).join('、')||'無'}});document.getElementById('log-empty').style.display='none';document.getElementById('log-table').style.display='table';document.getElementById('log-body').innerHTML=experimentLog.map((r,i)=>`<tr><td>${{i+1}}</td><td>${{r.time}}</td><td>${{r.pass}}</td><td>${{r.stop}}</td><td>${{r.length.toFixed(3)}} m</td><td>${{r.waypoint}}</td><td>${{r.fail}}</td></tr>`).join('')}});
  function download(name,text,type) {{const blob=new Blob([text],{{type}}),url=URL.createObjectURL(blob),a=document.createElement('a');a.href=url;a.download=name;a.click();setTimeout(()=>URL.revokeObjectURL(url),500)}}
  document.getElementById('download-config').addEventListener('click',()=>{{const config=structuredClone(DATA.rawSpec);config.parameters.forEach(p=>p.value=values[p.key]);download('system_design_lab.json',JSON.stringify(config,null,2),'application/json')}});
  document.getElementById('export-log').addEventListener('click',()=>{{const rows=[['index','time','pass','stop','path_length_m','waypoint_count','failed_gates'],...experimentLog.map((r,i)=>[i+1,r.time,r.pass,r.stop,r.length.toFixed(6),r.waypoint,r.fail])];download('system_design_experiments.csv',rows.map(r=>r.map(v=>`"${{String(v).replaceAll('"','""')}}"`).join(',')).join('\\n'),'text/csv;charset=utf-8')}});
  buildControls();update();window.addEventListener('resize',()=>Plotly.Plots.resize('design-view'));
}})();
</script></body></html>'''
    destination.write_text(document, encoding="utf-8")
    return destination
