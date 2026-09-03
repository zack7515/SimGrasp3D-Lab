"""建立可調參數的機械手、相機、軟管與障礙物學習工作台。"""

from __future__ import annotations

import json
from dataclasses import asdict
from html import escape
from pathlib import Path

import numpy as np

from simgrasp3d.models.motion import HoseMotionSpec
from simgrasp3d.models.specs import SceneSpec
from simgrasp3d.models.system_design import SystemDesignLabResult
from simgrasp3d.robot.kinematics import forward_kinematics
from simgrasp3d.visualization.assets import read_asset, write_plotly_asset

_CSS = read_asset("design_lab.css")
_JS = read_asset("design_lab.js")


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
    asset_root: str | Path | None = None,
) -> Path:
    """輸出可離線調參、記錄實驗並下載 JSON 的自包含學習頁。"""

    destination = Path(output_path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    plotly_src = write_plotly_asset(destination, asset_root)
    data = json.dumps(_payload(result, scene, motion), ensure_ascii=False).replace("</", "<\\/")
    title = "系統設計實驗室｜SimGrasp3D"
    document = f'''<!doctype html>
<html lang="zh-Hant"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>{escape(title)}</title><style>{_CSS}</style><script src="{plotly_src}"></script></head>
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
{_JS}</script></body></html>'''
    destination.write_text(document, encoding="utf-8")
    return destination
