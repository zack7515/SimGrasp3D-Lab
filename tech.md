# SimGrasp3D Lab 技術報告

- 版本：`0.12.0`
- 驗證基準：2026-09-03、固定亂數種子 `7515`
- 範圍：桌面 RGB-D 感知、六軸手臂、平行夾爪、柔性軟管、固定管路避障、MuJoCo 接觸物理與醫院情境教學。

> 本報告描述目前程式實際實作與固定基準輸出。所有門檻皆為未經實機校正的教學設定，不構成工業安全、醫療器材或臨床有效性證明。

## 1. 技術結論

SimGrasp3D Lab 將「看見物件、規劃抓取、避開障礙、搬運柔性物」拆成可單獨檢查的模擬層：

1. **系統設計層**先檢查相機覆蓋、深度不確定度、手臂可達性、夾爪尺寸、軟管彎曲與路徑淨空。
2. **幾何與感測層**建立場景點雲，以 z-buffer 生成理想 RGB-D，再加入量化、距離雜訊、孔洞與外參誤差。
3. **感知層**從觀測點雲估計桌面、物件包圍盒、法向與抓取候選。
4. **動作層**以 waypoint、六自由度 IK、機器人尺寸模型與解析碰撞檢查產生連續軌跡。
5. **物理層**以 MuJoCo cable 模型檢查軟管形變、接觸、摩擦與抓取約束。
6. **整合層**採 fail-closed：任一必要門檻失敗時不輸出控制命令，只保留診斷證據。

目前系統適合用來學習架構、資料流、參數敏感度與失敗模式；尚不能預測特定實體手臂、相機或軟管的真實成功率。

## 2. 系統邊界與假設

### 已納入

- 固定式針孔 RGB-D 相機與相機外參擾動。
- 六軸串聯手臂、平行夾爪、連桿膠囊碰撞體與簡化 URDF／SRDF。
- 桌面、剛體物件、固定管路與一條可變形軟管。
- 幾何軟管模型與 MuJoCo cable 接觸模型。
- 固定時間軸的七個醫院情境，用於比較風險與模型需求。

### 刻意未納入

- 真實相機標定、機器人 hand-eye calibration 與時間同步。
- 由 RGB 自動辨識物件；目前物件分割使用模擬 `instance_mask`。
- 完整 motion planning framework、連續 swept-volume 證明或動態重規劃。
- 軟管材料試驗、接觸參數辨識、致動器動力學與控制迴路延遲。
- 病患、無菌、人體組織或醫材的驗證模型。

## 3. 架構與資料流

```text
設計參數 ──> 六道快速設計閘門 ──> 場景設定
                                      │
                                      ├─> 幾何場景／點雲
                                      │       │
                                      │       └─> 理想 RGB-D ─> 感測誤差 ─> 觀測 RGB-D
                                      │                                  │
動作設定 ──> waypoint ─> 6D IK／碰撞 ─> 幾何軌跡                  感知管線
                            │             │                           │
物理設定 ────────────────────────────────> MuJoCo 軟管軌跡          │
                                          │                           │
                                          └────────> fail-closed <────┘
                                                         │
                                            授權重播命令或中止原因
                                                         │
                                       HTML 主頁／比較報告／JSON／NPZ
```

模組責任：

| 模組 | 主要責任 | 實作位置 |
|---|---|---|
| 場景與幾何 | 規格解析、表面取樣、座標轉換、距離計算 | [`scene/`](src/simgrasp3d/scene)、[`geometry/`](src/simgrasp3d/geometry) |
| 機器人 | FK、DLS IK、碰撞與描述檔 | [`robot/`](src/simgrasp3d/robot) |
| 感測 | 投影、z-buffer、深度誤差模型 | [`sensors/rgbd.py`](src/simgrasp3d/sensors/rgbd.py) |
| 感知 | 桌面、OBB、法向與抓取候選 | [`perception/geometry_pipeline.py`](src/simgrasp3d/perception/geometry_pipeline.py) |
| 動作與物理 | 軟管幾何、waypoint、MuJoCo 與醫院案例 | [`simulation/`](src/simgrasp3d/simulation) |
| 安全整合 | 門檻彙整、命令授權與 JSONL 重播 | [`integration/replay.py`](src/simgrasp3d/integration/replay.py) |
| 輸出與介面 | NPZ／JSON／PLY 與共用離線 runtime 的 HTML | [`io/`](src/simgrasp3d/io)、[`visualization/`](src/simgrasp3d/visualization) |

## 4. 座標、單位與可重現性

- 所有長度與距離以公尺表示，時間以秒表示，內部角度以弧度計算。
- JSON 姿態使用 `rpy_deg`；變換矩陣為齊次 `4 × 4` 矩陣。
- 相機光學座標採 `x` 向右、`y` 向下、`z` 向前。
- 深度圖中的 `0` 代表無有效觀測。
- 預設亂數種子為 `7515`，用於感測誤差與可重現案例。
- `outputs/` 是執行產物而非來源；相同軟體版本、依賴與設定才是重現前提。
- 所有頁面共用 `outputs/assets/plotly.min.js`，不連任何 CDN；離線瀏覽需整個 `outputs/` 目錄一起複製。

## 5. 方法

### 5.1 系統設計快速篩選

工作台提供 12 個可調參數：相機高度、側向位置、垂直 FOV、深度雜訊、手臂尺度、夾爪開度、軟管半徑、最小彎曲半徑、抓取位置、障礙尺度、安全邊界與抬升高度。

六道設計閘門如下：

| 閘門 | 檢查內容 | 失敗代表 |
|---|---|---|
| Camera coverage | 目標是否落在視錐與有效距離內 | 相機配置無法穩定觀測目標 |
| Depth uncertainty | 距離相關深度不確定度是否低於門檻 | 感測誤差可能吃掉安全裕度 |
| Reach reserve | 目標距離是否保留手臂可達餘量 | 目標接近或超出工作空間 |
| Gripper match | 夾爪開度是否匹配管徑 | 無法包覆或閉合不足 |
| Bend radius | 規劃曲率是否滿足最小彎曲半徑 | 可能造成過度彎折 |
| Path clearance | 抬升與搬運路徑是否保留安全淨空 | 可能碰撞固定管路 |

這一層是瀏覽器中的快速幾何估算，用來形成假設與找出明顯不可行的設計；它不等同完整 IK、碰撞或 MuJoCo 驗證。

為了在拖曳滑桿時即時更新，六道閘門在瀏覽器端以 JavaScript 重新實作了一次（[`visualization/static/design_lab.js`](src/simgrasp3d/visualization/static/design_lab.js)）。同一套幾何邏輯因此存在兩份實作：Python 管線是權威版本，JS 是估算版本。兩者共用的線段距離運算由 [`tests/test_design_lab_js.py`](tests/test_design_lab_js.py) 以 node 對隨機輸入比對；閘門公式本身仍只在 Python 端有測試，因此頁面上的數值應視為假設而非結論。

### 5.2 場景與機器人幾何

場景由 JSON 描述桌面、基本幾何物件、機器人關節、連桿尺寸、夾爪與相機。各表面以固定密度取樣為帶顏色與 instance ID 的點雲。機器人 FK 逐節組合關節變換，並以球體或膠囊近似連桿和夾爪碰撞體。

此方式計算快且容易觀察，但點雲表面不是 watertight mesh；幾何碰撞結果也不包含製造公差、纜線、護套或未建模治具。

### 5.3 RGB-D 投影與誤差

給定影像高度 `H` 與垂直視角 `vfov`，相機內參使用：

```text
fy = fx = (H / 2) / tan(vfov / 2)
cx = (W - 1) / 2
cy = (H - 1) / 2
u  = fx * x / z + cx
v  = fy * y / z + cy
```

世界點先轉入相機座標，捨棄 `z <= 0` 與畫面外點，再將像素座標四捨五入。多個點落在同一像素時，z-buffer 只保留最近點。

觀測深度的標準差為：

```text
sigma(z) = sigma_base + sigma_per_m2 * z^2
```

流程依序加入高斯深度雜訊、深度量化、隨機孔洞與相機外參擾動。外參誤差使用受擾動的實際相機姿態投影，但輸出仍保存名義姿態，因此重建到世界座標時會留下可量測偏差。

### 5.4 感知幾何

目前感知管線執行：

1. 以 RANSAC 找桌面平面並統計內點與殘差。
2. 使用模擬 instance mask 擷取各物件觀測點。
3. 計算 AABB 與去除極端值後的 OBB。
4. 以鄰域 PCA 估計表面法向。
5. 由物件頂面產生 top-down 抓取候選。
6. 依夾爪開度、支撐點數、法向與障礙淨空篩選候選。

使用 oracle mask 可將問題聚焦在 3D 幾何，但會高估真實感知系統表現。接入真實資料前，必須另行處理偵測、分割、遮擋、反光、深度失效與 domain gap。

### 5.5 軟管、IK 與路徑

幾何軟管由 49 個中心線節點表示，並依弧長重新取樣以維持節點分布。抓取後的移動使用 smoothstep 平滑位置、四元數 SLERP 平滑方向；每幀加入節長約束、簡化重力、障礙投影與桌面限制。

末端姿態 IK 使用 damped least squares：

```text
dq = J^T (J J^T + lambda^2 I)^-1 e
```

`J` 是以數值微分得到的 `6 × N` Jacobian；基準方向誤差權重為 `0.15 m/rad`、阻尼為 `0.02`，單次關節修正上限為 `5°`。位置與方向目標門檻分別為 `2 mm` 與 `1°`。

路徑規劃器先檢查相鄰關鍵幀的直線段。若不安全，會嘗試向上或側向加入單一 detour waypoint，搜尋步距 `0.05 m`、最大偏移 `0.35 m`。這是可解釋的教學 baseline，不保證能解多障礙、狹窄通道或需要多 waypoint 的問題。

### 5.6 MuJoCo 軟管物理

物理層使用 MuJoCo elasticity cable plugin 與 capsule 接觸幾何。基準設定為：

| 參數 | 基準值 |
|---|---:|
| timestep | `0.003 s` |
| settling time | `0.4 s` |
| bend stiffness | `4.0e6` |
| twist stiffness | `1.0e7` |
| density | `850 kg/m³` |
| friction | `1.0` |
| damping | `0.03` |
| armature | `0.002` |
| solver iterations | `60` |

夾取後以等效 attachment 將軟管跟隨末端軌跡，並記錄接觸力、穿透、抓取約束誤差、速度與長度守恆。材料常數目前只供相對比較，未由真實軟管的拉伸、彎曲、扭轉或摩擦試驗辨識。

### 5.7 Fail-closed 整合

整合層同時讀取感知、幾何軌跡與物理證據。基準必要門檻包括：

| 指標 | 門檻 |
|---|---:|
| 最大 IK 位置誤差 | `2 mm` |
| 最大 IK 方向誤差 | `1°` |
| 最小機器人淨空 | `5 mm` |
| 未解決路徑段 | `0` |
| 軟管穿透幀數 | `<= 1` |
| 抽樣最大接觸力 | `30 N` |
| 最大抓取約束誤差 | `15 mm` |
| 桌面高度誤差 | `5 mm` |
| 可行抓取候選 | 至少 `1` 個 |

只有全部必要證據通過，才會輸出離線 `replay.jsonl`。這份重播資料不是控制器，也不會連線到真實設備。

### 5.8 醫院情境

醫院套件以相同資料契約建立固定時間軸、ground truth／observation 軌跡、事件、指標與假設：

| ID | 情境 | 主要學習問題 |
|---|---|---|
| H1 | 檢體試管轉移 | 身分追蹤、姿態、容器搬運 |
| H2 | 無菌器械盤擺位 | 無菌區域與放置約束 |
| H3 | 病床旁多管路整理 | 多軟管拓樸、交纏與拉力 |
| H4 | 病區配送 | 移動路徑、門與人員互動 |
| H5 | 消毒覆蓋 | 表面覆蓋與重複掃描 |
| H6 | 假體超音波探頭掃描 | 接觸力、姿態與掃描覆蓋 |
| H7 | 導管路徑預覽 | 細長柔性物與路徑限制 |

這些案例是架構練習，不含人體組織模型、臨床工作流驗證、風險管理檔案或法規證據。

## 6. 資料契約

| 契約 | 版本 | 關鍵欄位 | 用途 |
|---|---|---|---|
| RGB-D frame | `1.0` | `rgb`、`depth`、`instance_mask`、`K`、`camera_to_world` | GT／觀測比較與點雲重建 |
| Trajectory | `3.0` | 時間、階段、TCP、關節、軟管節點、淨空、IK、物理 | 幾何與物理動畫 |
| System design result | `simgrasp3d.system_design_result.v1` | 參數、衍生量、六道閘門、preset | 工作台實驗交換 |
| Hospital case | `1.0` | 固定時間、階段、GT／觀測 tracks、signals、metrics、events | 醫院案例比較 |
| Replay | JSONL | frame、關節命令、夾爪命令、授權狀態 | 離線重播與稽核 |

NPZ 儲存大型數值陣列，JSON 儲存指標與結構化中繼資料，PLY 提供點雲互通，HTML 內嵌本次結果的數據與樣式、並以相對路徑載入共用的 Plotly runtime。資料讀寫實作位於 [`src/simgrasp3d/io/`](src/simgrasp3d/io)。

## 7. 固定基準結果

以下數值來自目前 `outputs/` 中的固定種子基準，目的是建立 regression reference，不是實機性能宣告。

### 7.1 場景與 RGB-D

| 指標 | 結果 |
|---|---:|
| 場景實體／表面點 | `20`／`32,596` |
| 影像像素 | `19,200` |
| GT／觀測有效深度 | `4,180`／`4,080` |
| 共同有效像素 | `3,536` |
| 深度 MAE／RMSE | `7.707 mm`／`19.841 mm` |
| 深度誤差 P95 | `60.519 mm` |
| 可見點保留率 | `84.59%` |
| 隨機孔洞 | `68` |
| 外參平移／旋轉擾動 | `1.259 mm`／`0.141°` |

### 7.2 動作、物理與感知

| 指標 | 結果 |
|---|---:|
| 幾何動畫 | `116 frames`、`9.6 s` |
| 關鍵幀／新增 waypoint／未解決路段 | `12`／`1`／`0` |
| 最小機器人淨空 | `5.590 mm` |
| 最大 IK 位置／方向誤差 | `1.766 mm`／`0.262°` |
| 機器人碰撞／不安全幀 | `0`／`0` |
| MuJoCo steps | `3,363` |
| 最大接觸力 | `19.717 N` |
| 最大抓取約束誤差 | `8.500 mm` |
| 最小接觸距離 | `-0.630 mm` |
| 非有限數值 | `0` |
| 桌面 RANSAC RMS | `3.268 mm` |
| 偵測物件／抓取候選／可行候選 | `3`／`6`／`1` |
| 選定抓取淨空 | `44.830 mm` |

基準整合通過全部必要門檻，輸出 `121` 筆事件、`116` 個命令幀；這只表示目前未校準門檻下的模擬授權成立。

### 7.3 物理敏感度

| 變體 | 相對基準形狀 RMS | 最大接觸力 | 最大抓取誤差 | 解讀 |
|---|---:|---:|---:|---|
| baseline | `0 m` | `19.717 N` | `8.500 mm` | 比較基準 |
| soft bend | `0.415 m` | `28.110 N` | `8.417 mm` | 彎曲剛度強烈影響形狀 |
| low friction | `0.216 m` | `17.469 N` | `8.536 mm` | 摩擦會改變滑移與形狀 |
| coarse timestep | `0.041 m` | `39.121 N` | `17.245 mm` | 較粗時間步顯著放大峰值與誤差 |

這組結果支持一個工程判斷：物理參數與數值設定必須做實物辨識及收斂測試，不能直接把單一模擬數值當成真實接觸力。

## 8. 執行與重現

建立 Python 3.11 以上環境並安裝鎖定於專案需求的套件：

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
simgrasp3d
```

完整流程依序建立場景、RGB-D、幾何軌跡、物理、感知、整合、醫院案例與 HTML。若只需較快的幾何流程：

```bash
simgrasp3d --no-simulate-physics --no-simulate-hospital
```

完整流程在單機 CPU 約 14 秒完成，輸出約 15 MB，其中 4.7 MB 是所有頁面共用的 Plotly runtime；其餘時間主要由 MuJoCo 積分佔用。

重要設定檔位於 [`configs/`](configs)。調整參數時建議一次只變更一項，保留產出 JSON，並比較指標與失敗碼，避免只依動畫外觀下結論。

## 9. 驗證策略

測試分為單元與跨模組回歸：

- 幾何轉換、表面取樣與場景組裝。
- RGB-D 投影、遮擋、誤差與檔案 round-trip。
- FK／IK、機器人碰撞、waypoint 與軟管連續性。
- MuJoCo 數值有限性、長度、接觸與敏感度。
- 桌面、物件幾何、抓取候選與 fail-closed。
- 醫院案例資料契約、主頁，以及頁面只載入存在的本機腳本。
- 工作台 JS 與 Python 的線段距離實作對拍（需要 node，未安裝時自動略過）。

執行方式：

```bash
pytest
ruff check .
```

測試能防止已知行為回歸，但不取代真實感測、材料、設備、臨床或安全驗證。

## 10. 已知失效模式與優先改善

| 優先度 | 目前限制 | 影響 | 建議下一步 |
|---|---|---|---|
| P0 | 無實機相機與 hand-eye 標定 | 世界座標與抓取位姿可能系統性偏移 | 先以公開 RGB-D／自錄標定板資料驗證投影與誤差 |
| P0 | 軟管參數未辨識 | 形狀、接觸力與滑移不可外推 | 建立彎曲、拉伸、摩擦小實驗並做參數擬合 |
| P0 | oracle instance mask | 感知結果高估真實表現 | 接入可評估的偵測／分割 baseline 與遮擋資料 |
| P1 | 單一 detour waypoint | 複雜障礙下可能找不到可行路徑 | 接入 MoveIt／OMPL 並驗證連續碰撞 |
| P1 | 簡化夾取 attachment | 無法表現局部壓縮與滑脫 | 建立夾爪接觸面、閉合控制與滑移指標 |
| P1 | 缺少控制與感測延遲 | 動畫不代表閉迴路穩定 | 加入控制頻率、延遲、追蹤誤差與 replanning |
| P2 | 工作台閘門在 JS 與 Python 各有一份實作 | 兩份公式可能分歧，頁面判定與完整管線不一致 | 把對拍範圍從線段距離擴大到六道閘門，或改由 Python 預先計算參數網格 |
| P2 | 醫院案例未建模人體與流程 | 不能做臨床或法規推論 | 與領域專家定義 task、hazard、acceptance evidence |

## 11. 證據與延伸閱讀

本報告的本地證據可由以下檔案重建：

- 設定：[`configs/`](configs)
- 基準執行入口：[`src/simgrasp3d/cli.py`](src/simgrasp3d/cli.py)
- 測試：[`tests/`](tests)
- 外部方法與資料集來源：[`research/references/sources.md`](research/references/sources.md)

主要外部技術基礎包括 [MuJoCo modeling 文件](https://mujoco.readthedocs.io/en/stable/modeling.html)、[MuJoCo elasticity plugins](https://mujoco.readthedocs.io/en/stable/programming/extension.html#elasticity-plugins)、[MoveIt URDF／SRDF 教學](https://moveit.picknik.ai/main/doc/examples/urdf_srdf/urdf_srdf_tutorial.html) 與 [GraspNet-1Billion](https://graspnet.net/)。研究來源清單會比本報告保留更完整的論文、官方文件與公開資料集連結。
