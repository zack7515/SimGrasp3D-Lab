# SimGrasp3D Lab

以 Python 建立的 3D 空間、點雲與機器手臂抓取模擬學習專案。專案目前不依賴實體設備，先用可重現的幾何場景理解座標轉換、機構尺寸、點雲取樣與互動式 3D 視覺化。

作者：`zack7515`

> 目前階段：幾何、點雲、RGB-D 感測與軟管運動學教學原型。已加入六自由度 IK、flange/TCP 工具標定、機器人尺寸碰撞及自動 waypoint；尚未加入接觸力、摩擦、材料參數、連續 swept-volume 或自動抓取策略，因此輸出不代表實機抓取結果。

## 目前可以做什麼

- 由 JSON 定義桌面、物件、六軸機械手、夾爪與虛擬 RGB-D 相機。
- 依公尺尺寸產生盒體、圓柱、球體及機械手表面點雲。
- 計算序列式機械手正向運動學與 TCP 世界座標。
- 以阻尼最小平方六自由度 IK 產生連續關節骨架，驗證 TCP 位置、姿態與關節限制。
- 分離 flange 與指尖 TCP offset，以實際連桿、關節、手掌與手指尺寸檢查桌面／管路距離。
- 使用保守工具包覆體檢查直線路徑，遇到風險時自動加入安全 waypoint。
- 模擬軟管夾取、抬升、繞管、搬運、放置與退回，分開統計機器人安全與軟管接觸。
- 在瀏覽器中旋轉、縮放及切換物件、機械手、座標軸與相機視錐。
- 將各實體與完整場景匯出為 ASCII PLY，供 Open3D、CloudCompare 等工具使用。
- 以 pinhole 相機和 z-buffer 產生可見 RGB、深度、instance mask 與彩色點雲。
- 注入深度量化、距離相關雜訊、隨機孔洞及相機外參偏移，比較 ground truth 與 observation。
- 使用固定 random seed 重現相同場景，並以測試驗證幾何與輸出行為。

更完整的相機方案、工業作法、抓取管線、論文與決策分析請閱讀 [研究與學習筆記](research/README.md)。

## 快速開始

需求：Python 3.11 以上。

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
```

產生預設場景並以瀏覽器開啟：

```bash
simgrasp3d --open
```

若不需要自動開啟瀏覽器：

```bash
simgrasp3d
```

也可以使用開發用入口：

```bash
python scripts/run_scene.py
```

## 執行結果

預設會產生：

```text
outputs/
├── simulation_report.html    # 世界、RGB-D 與連續動作的單頁報告
├── tabletop_scene.html       # 可離線開啟的互動式 3D 場景
├── hose_motion.html          # 可播放、暫停及逐幀拖曳的軟管動作動畫
├── point_clouds/
│   ├── complete_scene.ply    # 合併後的完整場景點雲
│   ├── blue_box.ply          # 個別物件點雲
│   ├── orange_cylinder.ply
│   ├── green_sphere.ply
│   └── ...                   # 機械手各連桿、關節與夾爪點雲
├── sensor/
│   ├── ground_truth_frame.npz     # 理想 RGB-D frame
│   ├── observation_frame.npz      # 含雜訊與孔洞的 RGB-D frame
│   ├── ground_truth_visible.ply   # 理想相機可見點雲
│   ├── observation_visible.ply    # 使用名義外參回投影的觀測點雲
│   ├── metrics.json               # 深度誤差、覆蓋率與外參擾動
│   └── rgbd_comparison.html       # RGB、深度與誤差互動比較頁面
└── motion/
    ├── trajectory.npz             # 逐幀 TCP、關節、軟管與距離資料
    └── metrics.json               # IK、碰撞、安全餘量與約束摘要
```

`outputs/` 是可重建產物，已由 Git 忽略，不會進入版本歷史。

建議優先開啟 `outputs/simulation_report.html`。A/B 雙畫面比較原始 3D 世界與 RGB-D ground truth／observation；C 區可播放軟管連續動作、拖曳時間軸並查看安全狀態，兩組完整量化指標都在同一頁。執行 `simgrasp3d --open` 時只會開啟這份整合報告。

## 專案結構

```text
simgrasp3d-lab/
├── configs/
│   ├── scenes/               # 場景、尺寸、姿態與相機設定
│   └── motions/              # 軟管、管路、關鍵幀與安全餘量設定
├── research/                 # 研究筆記、比較資料與來源索引
│   ├── data/                 # 感測方案與研究比較表
│   ├── references/           # 外部文件、論文及 benchmark 來源
│   └── README.md             # 3D 感知與機器人抓取研究筆記
├── scripts/                  # 開發用執行入口
├── src/simgrasp3d/
│   ├── geometry/             # 幾何取樣與 3D 齊次座標轉換
│   ├── io/                   # 點雲檔案輸出
│   ├── models/               # 場景設定資料模型與驗證
│   ├── robot/                # 機械手正向運動學與幾何建立
│   ├── scene/                # 場景載入與組合
│   ├── sensors/              # RGB-D 投影、z-buffer 與感測誤差
│   ├── simulation/           # 軟管時間序列與幾何約束求解
│   ├── visualization/        # Plotly 互動式 3D 視覺化
│   └── cli.py                # `simgrasp3d` 命令列流程
├── tests/                    # 幾何、取樣、場景與匯出測試
├── CONTRIBUTING.md           # 協作、隱私與提交規範
├── pyproject.toml            # 套件、相依套件與測試設定
└── README.md                 # 專案入口與使用說明
```

## 主要檔案與功能

| 路徑 | 功能 | 修改時機 |
|---|---|---|
| `configs/scenes/tabletop_demo.json` | 定義場景單位、seed、桌面、物件、機械手、夾爪及相機 | 調整尺寸、姿態、關節角或點數時 |
| `configs/motions/hose_extraction_demo.json` | 定義軟管中心線、固定管路、夾取節點、關鍵幀與安全餘量 | 練習不同抽取路徑或障礙配置時 |
| `src/simgrasp3d/models/specs.py` | 將 JSON 轉成具型別的設定物件，並檢查尺寸、顏色、單位及必要欄位 | 新增場景欄位或物件種類時 |
| `src/simgrasp3d/models/motion.py` | 定義軟管情境、關鍵幀、`TrajectoryFrame` 與 `TrajectoryData` | 擴充運動時間序列欄位時 |
| `src/simgrasp3d/geometry/transforms.py` | 建立平移、旋轉、姿態矩陣，轉換點座標並對齊圓柱方向 | 擴充座標系或姿態運算時 |
| `src/simgrasp3d/geometry/collision.py` | 計算線段、膠囊體與水平桌面的解析式有號距離 | 擴充碰撞幾何時 |
| `src/simgrasp3d/geometry/sampling.py` | 對盒體、圓柱與球體表面取樣，提供點雲容器與邊界計算 | 新增幾何形狀或取樣方法時 |
| `src/simgrasp3d/robot/kinematics.py` | 計算 FK、位置／六自由度 IK、flange、TCP 與機械手點雲 | 修改機構模型、工具標定或 IK 時 |
| `src/simgrasp3d/robot/collision.py` | 以膠囊體包覆連桿、關節、手掌與手指，計算環境距離 | 修改機器人碰撞模型時 |
| `src/simgrasp3d/scene/builder.py` | 載入設定、建立所有實體、相機座標與視錐，組成完整場景 | 加入感測、燈光或場景元素時 |
| `src/simgrasp3d/sensors/rgbd.py` | 定義 `RGBDFrame`，執行投影、z-buffer、回投影及深度／外參誤差模擬 | 修改相機模型或真實資料轉接格式時 |
| `src/simgrasp3d/simulation/hose_motion.py` | 產生軟管固定節長、附著、重力近似、障礙投影與逐幀距離 | 換路徑規劃器或物理求解器時 |
| `src/simgrasp3d/simulation/waypoint_planner.py` | 找出不安全 TCP 直線並搜尋單一繞行點 | 擴充多 waypoint 或採樣式規劃時 |
| `src/simgrasp3d/visualization/plotly_viewer.py` | 建立互動式圖層、座標軸、hover 資訊與 HTML | 改善視覺呈現或除錯資訊時 |
| `src/simgrasp3d/visualization/rgbd_viewer.py` | 比較理想深度、觀測深度、絕對誤差與 RGB | 分析感測品質時 |
| `src/simgrasp3d/visualization/motion_viewer.py` | 建立播放、暫停、時間軸、安全狀態與動作階段 3D 動畫 | 調整連續動作教學畫面時 |
| `src/simgrasp3d/visualization/simulation_report.py` | 將原始世界、感測比較、連續動作與全部指標組成單頁報告 | 調整整合報告資訊或版面時 |
| `src/simgrasp3d/io/point_cloud.py` | 清理檔名並輸出個別或合併的 ASCII PLY | 支援 PCD、二進位 PLY 或其他格式時 |
| `src/simgrasp3d/io/rgbd_frame.py` | 讀寫壓縮 NPZ、可見點雲及 JSON 模擬指標 | 匯入真實資料或擴充 schema 時 |
| `src/simgrasp3d/io/trajectory.py` | 匯出不含 pickle 的逐幀 NPZ 與動作指標 JSON | 接入 notebook、物理引擎或資料分析時 |
| `src/simgrasp3d/cli.py` | 串接設定載入、場景建立、HTML/PLY 輸出與瀏覽器開啟 | 新增命令列參數或工作流程時 |
| `scripts/run_scene.py` | 尚未安裝 CLI 時的簡易執行入口 | 本機開發與快速除錯時 |
| `tests/` | 驗證轉換矩陣、表面取樣、固定 seed、場景結構與 PLY 標頭 | 修改核心幾何或輸出邏輯後 |
| `research/README.md` | 彙整 3D 感知、抓取方法、工業實務與模擬規劃 | 閱讀或補充研究筆記時 |
| `research/data/*.csv` | 保存相機方法、工程評分與公開實驗的可機讀資料 | 更新研究比較與證據時 |
| `research/references/sources.md` | 保存官方文件與研究來源 | 新增或查核外部資料時 |

## 程式流程

```mermaid
flowchart LR
    A[場景 JSON] --> B[設定解析與驗證]
    B --> C[幾何表面取樣]
    B --> D[機械手正向運動學]
    C --> E[完整 SceneData]
    D --> E
    E --> F[Plotly HTML]
    E --> G[個別與完整 PLY]
    E --> H[Pinhole 投影與 z-buffer]
    H --> I[Ground truth RGB-D]
    H --> J[雜訊與外參誤差]
    J --> K[Observation RGB-D]
    I --> L[誤差指標與比較 HTML]
    K --> L
    B --> N[軟管與動作關鍵幀]
    N --> O[自動 waypoint 與六自由度 IK]
    O --> R[機器人尺寸碰撞與軟管接觸]
    R --> P[TrajectoryFrame 時間序列]
    P --> Q[可播放 3D 動畫與 NPZ]
    E --> M[單頁驗證報告]
    L --> M
    Q --> M
```

完整場景 PLY 仍是所有取樣表面的 ground truth；`outputs/sensor/` 則是經相機投影與 z-buffer 後的可見資料。第一版使用離散表面點投影，不是 mesh triangle rasterization，因此影像填充率會受 `point_count` 影響，不能把空白像素全都解讀成真實相機失效。

### RGBDFrame v1.0

每個 `.npz` 使用同一資料契約：

| 欄位 | dtype / shape | 定義 |
|---|---|---|
| `rgb` | `uint8 [H,W,3]` | 對齊深度的 RGB |
| `depth_m` | `float32 [H,W]` | 米制 z-depth，`0` 表示無效 |
| `instance_mask` | `uint16 [H,W]` | `0` 為背景，其餘值對應 `instance_names` |
| `intrinsics` | `float64 [3,3]` | pinhole 相機內參 `K` |
| `camera_to_world` | `float64 [4,4]` | 名義光學相機座標到世界座標的轉換 |
| `instance_names` | Unicode array | instance id 到名稱的對照 |
| `frame_id` | Unicode scalar | `ground_truth` 或 `observation` |

光學相機採 OpenCV 慣例：x 向右、y 向下、z 向前。`observation` 的 `camera_to_world` 刻意保存系統認知的名義外參；實際擾動後的外參另存於 `metrics.json`，因此其回投影點雲能呈現校正誤差。

### Motion trajectory v2.0

`outputs/motion/trajectory.npz` 是目前幾何求解器與未來物理引擎共用的逐幀資料契約：

| 欄位 | dtype / shape | 定義 |
|---|---|---|
| `time_s` | `float64 [T]` | 每幀模擬時間 |
| `phase` | Unicode `[T]` | 待機、接近、夾取、抬升、避障、放置等階段 |
| `tcp_position` | `float64 [T,3]` | 規劃的 TCP 世界座標 |
| `tcp_rpy_deg` / `tcp_rotation` | `[T,3]` / `[T,3,3]` | 規劃的 TCP 姿態 |
| `tool_frame` | `float64 [T,4,4]` | 六自由度 IK 回算的 TCP 世界姿態 |
| `joint_angles_deg` | `float64 [T,J]` | 六自由度 IK 求得的關節角 |
| `robot_joint_positions` | `float64 [T,J+1,3]` | 動畫用機械臂骨架 |
| `gripper_opening_m` | `float64 [T]` | 平行夾爪開口 |
| `attached` | `bool [T]` | 軟管夾取節點是否附著於 TCP |
| `hose_nodes` | `float64 [T,N,3]` | 軟管中心線節點 |
| `minimum_clearance_m` | `float64 [T]` | 機器人所有包覆體到桌面／管路的最小距離 |
| `link_clearance_m` / `gripper_clearance_m` | `float64 [T]` | 連桿與夾爪分項最小距離 |
| `hose_clearance_m` | `float64 [T]` | 軟管到固定管路的距離，獨立於機器人安全判定 |
| `ik_position_error_m` | `float64 [T]` | TCP 目標與 FK 回算位置的誤差 |
| `ik_orientation_error_deg` | `float64 [T]` | TCP 目標與 FK 回算姿態的角度誤差 |
| `hose_length_ratio` | `float64 [T]` | 當前中心線總長相對初始總長 |

第二版以姿態矩陣求六自由度 IK，並用膠囊體保守包覆機器人尺寸。自動 waypoint 目前每段最多插入一點，不能取代 OMPL／MoveIt 等完整規劃器；軟管仍採離散節點與固定節長近似。後續接入物理引擎時應維持這些輸出欄位，額外增加接觸力、速度、材料參數與求解器資訊。

`outputs/motion/metrics.json` 另保存規劃器參數、原始／自動產生標記及完整規劃後關鍵幀，可用來重建「為何繞行」的決策過程。

### 第一個固定測試情境

- 狀態：固定式 eye-to-hand 相機觀察靜態桌面、三個基本物件與六軸機械手。
- 輸入：`tabletop_demo.json` 的公尺制完整表面點雲，`seed=7515`。
- 相機：160×120、垂直 FOV 52°、近裁切 0.12 m、遠裁切 1.75 m。
- Ground truth：名義外參、無深度雜訊，z-buffer 保留每個像素最近點。
- Observation：實際相機姿態加入外參擾動，深度加入距離相關雜訊、1 mm 量化及 2% 隨機孔洞；回投影仍使用名義外參。
- 測試目的：先隔離相機資料流程，不在同一輪混入桌面估計、抓取候選、IK 或接觸物理。

使用目前設定執行 `simgrasp3d` 的固定結果：

| 指標 | 結果 |
|---|---:|
| 影像像素 | 19,200 |
| Ground-truth 有效像素 | 4,180（21.77%） |
| Observation 有效像素 | 4,080（21.25%） |
| 共同有效像素 | 3,536（保留 84.59%） |
| 注入深度孔洞 | 68 |
| 外參平移誤差範數 | 1.259 mm |
| 外參旋轉誤差範數 | 0.141° |
| 共同像素深度 MAE | 7.71 mm |
| 共同像素深度 RMSE | 19.84 mm |
| 共同像素絕對誤差 P95 | 60.52 mm |

較大的邊界誤差同時包含外參偏移後「不同表面落到同一像素」的影響，不等同於相機軸向 noise standard deviation。完整數值以每次輸出的 `outputs/sensor/metrics.json` 為準。

### 第二個固定測試情境：軟管抽取

- 狀態：桌上軟管穿過三根固定管路附近，六軸手臂執行預抓取、下降、閉爪、抬升、繞管、抽離、搬運、放置與退回。
- 輸入：49 個等弧長軟管中心線節點、11 個關鍵幀、12 Hz、5 mm 教學安全餘量。
- 求解：阻尼最小平方六自由度 IK、膠囊體碰撞與自動 waypoint；軟管使用固定節長、重力、夾取節點附著與圓柱障礙投影。
- 目的：先驗證資料格式、座標、可達性、連續性與距離警示，再以相同 `TrajectoryData` 介面替換成接觸物理。

目前固定結果：

| 指標 | 結果 | 解讀 |
|---|---:|---|
| 動作長度 | 116 幀 / 9.6 s | 可播放及逐幀拖曳 |
| 原始／規劃後關鍵幀 | 11 / 12 | 自動插入 1 個安全 waypoint |
| 最大 IK 位置／姿態誤差 | 1.77 mm / 0.26° | 全部幀通過 2 mm / 1° 容差 |
| 機器人最小環境距離 | 5.59 mm | 包含連桿、關節、手掌、手指、桌面與管路 |
| 機器人警示／碰撞幀 | 0 / 0 | 全部幀高於 5 mm 安全餘量 |
| 軟管接觸／穿透幀 | 62 / 0 | 接觸允許，穿透才是幾何失敗 |
| 最大軟管長度誤差 | 0.73% | 幾何約束品質通過目前 1% 測試門檻 |

機器人與軟管採不同判定：機器人必須保留安全餘量；軟管在抽取時可以接觸管路，但不應穿透。數值以 `outputs/motion/metrics.json` 為準。

## 調整場景

預設設定位於 [`configs/scenes/tabletop_demo.json`](configs/scenes/tabletop_demo.json)。所有長度均使用公尺，姿態角使用度數：

- `pose.xyz`：實體中心在世界座標中的 `(x, y, z)`。
- `pose.rpy_deg`：依 roll、pitch、yaw 定義姿態。
- `dimensions` / `size`：物件或桌面的實際尺寸。
- `joint_axis` / `joint_angle_deg`：旋轉關節軸與角度。
- `joint_limits_deg`：IK 可使用的關節角下限與上限。
- `translation`：目前關節旋轉後，到下一關節的局部位移。
- `opening`：平行夾爪開口。
- `tcp_offset`：從最後一個 flange 到指尖 TCP 的局部座標偏移。
- `point_count` / `points_per_link`：視覺化點數；越高越細緻，也越耗記憶體與瀏覽器效能。
- `seed`：控制隨機表面取樣；相同設定與 seed 會產生相同點雲。
- `camera.width` / `camera.height`：RGB-D 輸出解析度，需與 `aspect_ratio` 一致。
- `camera.noise.depth_quantization_m`：深度量化間距。
- `camera.noise.axial_noise_std_base_m` / `axial_noise_std_per_m2`：基礎與距離平方相關的軸向雜訊。
- `camera.noise.dropout_probability`：有效深度被改成孔洞的機率。
- `camera.noise.extrinsic_*`：相機平移與旋轉外參的標準差。

連續動作位於 [`configs/motions/hose_extraction_demo.json`](configs/motions/hose_extraction_demo.json)：

- `hose.control_points` / `node_count`：初始中心線與離散解析度。
- `hose.grasp_node_index`：閉爪後附著於 TCP 的中心線節點。
- `obstacles`：固定管路的起點、終點與半徑。
- `keyframes`：每階段終點的 TCP 位置、`tcp_rpy_deg` 姿態、時間、夾爪開口與附著狀態。
- `waypoint_planner`：工具包覆半徑、搜尋步距與最大繞行距離；目前每段最多自動插入一點。
- `safe_clearance_m`：機器人橘色近接警示門檻，不是碰撞容差。
- `collision_tolerance_m`：離散約束的數值穿透容差；超過才計為碰撞。
- `constraint_iterations`：越高越能維持管長，但 CPU 時間也越高；批次訓練不應產生 HTML。

使用其他設定檔與輸出位置：

```bash
simgrasp3d \
  --config configs/scenes/tabletop_demo.json \
  --output outputs/custom_scene.html \
  --point-cloud-dir outputs/custom_clouds \
  --report-output outputs/custom_report.html \
  --motion-config configs/motions/hose_extraction_demo.json \
  --motion-output outputs/custom_motion.html
```

只建立 HTML、不匯出 PLY：

```bash
simgrasp3d --no-export-point-clouds
```

略過 RGB-D 感測模擬：

```bash
simgrasp3d --no-simulate-rgbd
```

只練習靜態場景與 RGB-D、略過連續動作：

```bash
simgrasp3d --no-simulate-motion
```

查看所有參數：

```bash
simgrasp3d --help
```

## 測試

```bash
pytest
```

測試範圍包括：

- 平移、旋轉、四元數姿態往返／插值與點座標轉換。
- 線段、膠囊體及桌面有號距離。
- 盒體、圓柱及球體表面取樣是否符合幾何邊界。
- 固定 seed 的場景是否可重現。
- 關節、連桿與 TCP 結構是否一致。
- 六自由度 IK 是否在關節限制內到達每幀 TCP 位置與姿態。
- 軟管夾取節點是否跟隨 TCP、總長誤差是否低於 1%。
- 連桿／夾爪安全距離、軟管接觸／穿透、自動 waypoint 與逐幀資料 shape。
- 物件是否位於桌面上方。
- PLY 匯出格式是否具有正確標頭。
- z-buffer 是否保留同像素最近點，以及深度能否正確回投影。
- 深度雜訊、量化、外參擾動是否固定 seed 重現。
- `RGBDFrame` 壓縮 NPZ 是否能無損往返。
- 單頁報告是否同時包含原始世界、感測畫面、連續動作與所有 metrics，且不依賴外部 script。

## 資料與文件

| 文件 | 內容 |
|---|---|
| [research/README.md](research/README.md) | 2D、RGB-D、工業 3D 相機、座標標定、抓取方法、工業實務、可行性與研究整理 |
| [camera_methods.csv](research/data/camera_methods.csv) | 各類相機與感測方法的能力、限制和適用情境 |
| [decision_matrix.csv](research/data/decision_matrix.csv) | 感測方案評分、權重與加權比較 |
| [experiments.csv](research/data/experiments.csv) | 公開研究及實機實驗索引 |
| [real_world_datasets.csv](research/data/real_world_datasets.csv) | 可用真實 RGB-D／點雲資料、測試用途與限制 |
| [sources.md](research/references/sources.md) | 官方文件、論文與 benchmark 來源清單 |
| [CONTRIBUTING.md](CONTRIBUTING.md) | 公開協作、隱私、模擬結果與 commit 規範 |

## 已知限制與開發順序

目前已完成第一版相機投影、點式 z-buffer、深度／外參誤差、`RGBDFrame`，以及軟管時間序列、六自由度 IK、flange/TCP 標定、逐幀機器人膠囊體碰撞與單 waypoint 安全化。尚未包含 mesh rasterization、真實資料集 adapter、桌面分割、連續 swept-volume／自碰撞、完整取樣式路徑規劃、抓取候選、接觸物理及 ROS 2 整合。

建議依下列順序擴充：

1. **已完成第一版**：世界點投影、z-buffer、可見深度與 RGB-D 點雲。
2. **已完成第一版**：深度量化、距離相關雜訊、孔洞、外參誤差與比較指標。
3. **已完成第一版**：軟管幾何時間序列、關鍵幀狀態機、夾取附著、固定管路距離與動畫。
4. **已完成第一版**：六自由度 IK、flange/TCP offset、連桿／夾爪膠囊體碰撞與單 waypoint 安全化；目前機器人警示／碰撞皆為 0 幀。
5. **下一步**：以 MuJoCo 的一維 cable/flex 模型取代幾何約束，加入重力、摩擦、彎曲／扭轉、夾持接觸與 solver sensitivity 測試。
6. 再為 `RGBDFrame` 實作桌面分割、物件點雲、AABB／OBB、法向與抓取候選，讓感知結果驅動相同動作管線。
7. 最後視高精度柔性體、ROS 2 或大量合成資料需求，評估 SOFA、Gazebo 或 Isaac Sim，並接入真實資料 replay。

大量場景訓練時，應將點雲、深度影像與 HTML 視覺化分流：訓練資料採批次及壓縮格式，HTML 僅用於抽樣檢查，避免不必要的 RAM/VRAM 與磁碟開銷。若未來部署到 Jetson，感知模型再依實測瓶頸評估 FP16、TensorRT 與點雲降採樣。

## 協作

提交前請閱讀 [CONTRIBUTING.md](CONTRIBUTING.md)，並啟用 repository hooks：

```bash
git config core.hooksPath .githooks
```

本專案不提交憑證、本機設定、私人資訊、設備序號、大型模型權重或工具署名資訊；所有結果需清楚標示為模擬或實機資料。
