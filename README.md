# SimGrasp3D Lab

以純模擬方式學習 3D 空間感知、機器手臂、RGB-D、柔性軟管抓取、避障與安全驗證的 Python 專案。適合沒有實體設備、想先理解一套抓取系統如何配置與調參的學習者。

作者：[`zack7515`](https://github.com/zack7515)

> 本專案所有門檻與結果都只適用於固定設定的模擬教學，不代表實機抓取成功、工業安全認證、醫療器材符合性或臨床有效性。

## 可以學到什麼

- 配置六軸機械手、平行夾爪、固定 RGB-D 相機、軟管、桌面與管路障礙。
- 調整 12 個系統參數，觀察視錐、工作空間、管徑匹配、彎曲半徑與路徑淨空。
- 將世界點投影成 RGB-D，加入深度量化、距離雜訊、孔洞與相機外參誤差。
- 練習六自由度 IK、機器人尺寸碰撞、waypoint、MuJoCo cable 接觸物理與參數敏感度。
- 由模擬點雲估計桌面、AABB／OBB、表面法向與抓取候選。
- 使用 fail-closed 閘門決定是否產生離線控制重播。
- 以七個醫院情境理解模型複雜度與風險邊界。

## 快速開始

需求：Python 3.11 以上。預設流程可在 CPU 執行，不需要 CUDA 或 GPU。

```bash
git clone https://github.com/zack7515/SimGrasp3D-Lab.git
cd SimGrasp3D-Lab

python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
```

執行完整模擬：

```bash
simgrasp3d --open
```

若尚未安裝命令列入口，可改用：

```bash
python scripts/run_scene.py
```

完成後從以下主頁開始：

```text
outputs/index.html
```

`outputs/` 是可重建產物，已由 Git 忽略。所有頁面共用 `outputs/assets/plotly.min.js`，不連 CDN，複製整個 `outputs/` 目錄即可離線瀏覽。

## 使用方式

1. 進入 `outputs/system_design_lab.html`，用基準設計與故障 preset 理解參數間的因果關係。
2. 一次只修改一個滑桿，查看六道設計閘門與 3D 包絡如何變化。
3. 進入 `outputs/simulation_report.html`，檢查 RGB-D、IK、碰撞、物理、感知與命令證據。
4. 進入 `outputs/hospital/index.html`，依 H1～H7 觀察不同醫院情境需要補上的模型。
5. 下載工作台產生的 JSON，改回 `configs/` 後重新執行 `simgrasp3d`，用完整管線驗證工作台的估算。

若只想先產生幾何、RGB-D 與系統設計頁面，可略過較耗時的物理與醫院案例：

```bash
simgrasp3d --no-simulate-physics --no-simulate-hospital
```

查看所有 CLI 選項：

```bash
simgrasp3d --help
```

## 主要入口

| 路徑 | 用途 |
|---|---|
| `outputs/index.html` | 統一學習主頁與本次執行摘要 |
| `outputs/system_design_lab.html` | 可調整手臂、相機、夾爪、軟管與避障參數的工作台 |
| `outputs/simulation_report.html` | 世界、感測、動作、物理、感知與控制整合報告 |
| `outputs/hospital/index.html` | 七個醫院機器人模擬案例 |
| `outputs/system_design_result.json` | 系統設計基準與故障 preset 評估結果 |

## 主要設定

| 設定檔 | 控制內容 |
|---|---|
| `configs/learning/system_design_lab.json` | 可調參數、設計閘門與故障 preset |
| `configs/scenes/tabletop_demo.json` | 桌面、物件、手臂、夾爪、相機與雜訊 |
| `configs/motions/hose_extraction_demo.json` | 軟管、固定管路、關鍵幀與路徑安全距離 |
| `configs/physics/hose_mujoco_baseline.json` | MuJoCo 材料、摩擦、求解器與敏感度案例 |
| `configs/perception/rgbd_geometry_baseline.json` | 平面、包圍盒、法向與抓取候選參數 |
| `configs/integration/fail_closed_baseline.json` | IK、碰撞、接觸力與命令授權門檻 |
| `configs/hospital/hospital_learning_suite.json` | H1～H7 案例與未校準教學門檻 |

所有長度使用公尺，姿態設定使用 `rpy_deg`。修改設定後重新執行 `simgrasp3d` 即可重建結果。

## 專案結構

```text
SimGrasp3D-Lab/
├── configs/                 # 場景、動作、物理、感知與安全設定
├── research/                # 相機、資料集、方法比較與外部來源
├── scripts/                 # 開發用執行入口
├── src/simgrasp3d/
│   ├── geometry/            # 幾何轉換、取樣與解析距離
│   ├── robot/               # FK、IK、碰撞與 URDF/SRDF
│   ├── sensors/             # RGB-D 投影與誤差模型
│   ├── perception/          # 桌面、OBB、法向與抓取候選
│   ├── simulation/          # 軟管、waypoint、MuJoCo 與醫院案例
│   ├── integration/         # fail-closed 與離線重播
│   ├── visualization/       # 主頁、工作台、動畫與報告
│   │   └── static/          # 頁面 CSS 與瀏覽器端 JS
│   └── io/                  # JSON、JSONL、NPZ 與 PLY 輸出
├── tests/                   # 單元與跨模組回歸測試
├── tech.md                  # 架構、方法、資料契約與基準技術報告
└── pyproject.toml           # 套件版本、依賴與 CLI 入口
```

## 測試

```bash
pytest
ruff check .
```

測試涵蓋座標轉換、RGB-D、IK、碰撞、軟管連續性、MuJoCo、感知、fail-closed、醫院案例與離線 HTML。
`tests/test_design_lab_js.py` 另外用 node 比對工作台 JS 與 Python 的線段距離實作，避免兩份幾何程式碼分歧；
未安裝 node 時該項自動略過。

## 已知限制

- 感知分割使用模擬 `instance_mask` 作為 oracle baseline，尚未接入真實 RGB-D 資料。
- 軟管材料與摩擦參數尚未用實物校正。
- waypoint 規劃器不是完整的連續 swept-volume 或 OMPL 規劃器。
- 系統設計工作台的即時閘門是瀏覽器端的快速估算，與 Python 完整管線是兩份實作；頁面數值用來形成假設，結論仍以重新執行管線為準。
- URDF／SRDF、命令與醫院案例皆為簡化教學模型，不會連接或控制真實設備。
- 所有 `PASS` 只表示通過目前 JSON 中未校準的模擬門檻。

## 文件

- [技術報告](tech.md)：架構、方法、公式、資料契約、基準結果與限制。
- [研究筆記](research/README.md)：相機方案、工業作法、公開資料集與延伸技術。
- [參考來源](research/references/sources.md)：官方文件、論文與 benchmark。
- [協作規範](CONTRIBUTING.md)：隱私、模擬結果與 commit 規則。
