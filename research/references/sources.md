# 來源與證據索引

> 查核日期：2026-09-01。優先使用官方文件、作者專案頁、正式論文與公開 benchmark。廠商資料只用來說明感測原理與代表性工程條件，不把單一廠商規格外推成所有產品表現。

| 分類 | 來源 | 本報告採用的主張 |
|---|---|---|
| 幾何/標定 | [OpenCV Camera Calibration and 3D Reconstruction](https://docs.opencv.org/4.13.0/d9/d0c/group__calib3d.html) | 內參、PnP、雙目與 hand-eye 的標準介面 |
| 機器人模型 | [MoveIt URDF and SRDF](https://moveit.picknik.ai/main/doc/examples/urdf_srdf/urdf_srdf_tutorial.html) | URDF/SRDF、碰撞 mesh、end effector、自碰撞矩陣 |
| 世界模型 | [MoveIt Perception Pipeline](https://moveit.picknik.ai/main/doc/examples/perception_pipeline/perception_pipeline_tutorial.html) | 點雲/深度圖、TF 與 OctoMap 整合 |
| 即時閉迴路 | [MoveIt Realtime Servo](https://moveit.picknik.ai/main/doc/examples/realtime_servo/realtime_servo_tutorial.html) | visual servo、奇異點、碰撞、平滑與關節限制 |
| 手眼標定 | [Zivid Hand-Eye Calibration API](https://support.zivid.com/en/latest/camera/api-reference/hand-eye-calibration.html) | eye-in-hand/eye-to-hand 的 4×4 transform 定義與 residual |
| 手眼驗證 | [Zivid Touch Test](https://support.zivid.com/en/v2.8/academy/applications/hand-eye/hand-eye-calibration-verification-via-touch-test.html) | 用實際 TCP 接觸驗證手眼結果 |
| 主動雙目 | [RealSense D400 Datasheet](https://dev.realsenseai.com/download/42003/) | 左右影像視差與可選 IR projector 的工作原理 |
| ToF | [Basler ToF Camera Technology](https://docs.baslerweb.com/tof-camera-technology) | ToF 原理與環境光、multipath、反射率等風險 |
| 結構光 | [Zivid FAQ](https://support.zivid.com/en/latest/camera/support/faq.html) | 時序結構光與工業 3D 的代表用途/精度概念 |
| 動態結構光 | [Photoneo MotionCam-3D](https://www.photoneo.com/kb/MC-M-GEN2) | 平行結構光、靜態與動態模式 |
| 工程效能 | [Zivid ROI Production Preparation](https://support.zivid.com/en/latest/camera/academy/applications/piece-picking/prepare-for-production.html) | ROI 可降低擷取與下游處理成本 |
| 軟管物理 | [MuJoCo Modeling — Composite Objects / Flex](https://mujoco.readthedocs.io/en/3.6.0/modeling.html) | cable、flex 與一維柔性物體的模型選項及限制 |
| 高保真細長體 | [SOFA Supported Plugins — BeamAdapter](https://sofa-framework.github.io/doc/plugins/suported-plugins-list/) | Kirchhoff rod／beam 類細長柔性體插件定位 |
| GPU 柔性體 | [Isaac Sim Physics](https://docs.isaacsim.omniverse.nvidia.com/latest/physics/index.html) | PhysX、deformable 與機器人物理模擬入口 |
| 工業線纜規劃 | [Siemens Kineo Flexible Cables](https://www.siemens.com/en-us/products/plm-components/kineo/flexible-cables/) | 工業線纜路徑、干涉與數位驗證的代表性產品方向 |
| 工業線纜模擬 | [Fraunhofer IPS Cable Simulation](https://www.itwm.fraunhofer.de/en/departments/mf/products-and-services/ips.html) | 工業裝配中的電纜／軟管模擬與驗證應用 |
| 深度抓取 | [Dex-Net 2.0 Paper](https://goldberg.berkeley.edu/pubs/dex-net-2.0-Camera-Ready-RSS-2017.pdf) | depth-based grasp ranking、實機條件與原文成功率 |
| 抓取 benchmark | [GraspNet-1Billion Paper](https://openaccess.thecvf.com/content_CVPR_2020/papers/Fang_GraspNet-1Billion_A_Large-Scale_Benchmark_for_General_Object_Grasping_CVPR_2020_paper.pdf) | 資料規模、RGB-D、6D pose/grasp 標註 |
| 真實 RGB-D 資料 | [OCID 官方資料頁](https://www.acin.tuwien.ac.at/vision-for-robotics/software-tools/object-clutter-indoor-dataset/) | RGB、depth、2D label mask 與標註點雲內容 |
| 真實 RGB-D / 6D pose | [BOP Datasets](https://bop.felk.cvut.cz/datasets/) | YCB-V 等資料的真實影像、物件模型、mask、6D pose 與授權 |
| 透明物深度 | [ClearGrasp 官方 repository](https://github.com/Shreeyak/cleargrasp) | 真實／合成資料下載、ground-truth depth 與透明物件感測限制 |
| 真實物件模型 | [YCB Object and Model Set](https://ycb-benchmarks.s3.amazonaws.com/index.html) | 物件 mesh、點雲、多視角 RGB/RGB-D 與資料授權 |
| 6D grasp | [Contact-GraspNet](https://github.com/NVlabs/contact_graspnet) | raw point cloud 到 6-DoF grasp、硬體需求與 segmentation 建議 |
| 7D grasp | [AnyGrasp Paper](https://arxiv.org/abs/2212.08333) | 靜態/動態 grasp、清箱與 picks/hour 實驗數字 |
| RGB 6D pose | [MegaPose](https://megapose6d.github.io/) | novel CAD object 的 RGB pose estimation 與實機抓取流程 |
| RGB-D 6D pose | [FoundationPose](https://nvlabs.github.io/FoundationPose/) | CAD/reference-based 6D pose estimation and tracking |
| 單眼 grasp | [MonoGraspNet](https://sites.google.com/view/monograsp/about) | 單張 RGB 的 6-DoF grasp pipeline |
| 單眼深度 | [Depth Anything V2](https://depth-anything-v2.github.io/) | relative/metric depth 模型、資料規模與模型大小 |
| 透明物 | [TransCG](https://github.com/galaxies99/transcg) | 透明物真實 RGB-D 深度補全資料與抓取 baseline |
| 6D pose benchmark | [BOP Challenge 2024](https://bop.felk.cvut.cz/challenges/bop-challenge-2024/) | seen/unseen、model-based/model-free tracks 與新資料集 |
| VLA | [RT-2](https://deepmind.google/blog/rt-2-new-model-translates-vision-and-language-into-action/) | 6,000+ trials 與未見情境成效 |
| VLA | [Octo](https://octo-models.github.io/) | 9 個實機 setup、跨 embodiment policy |
| VLA | [OpenVLA](https://openvla.github.io/) | 970k episodes、7B 參數、實機與 fine-tuning 評估 |
| VLA | [OpenPI / π0](https://www.pi.website/blog/openpi) | 開源 general-purpose robot foundation model |
| VLA | [π0.7 Technical Report](https://www.pi.website/download/pi07.pdf) | 2026 多平台、多任務與組合泛化研究方向 |

## 解讀限制

- 不同論文的「success」定義、物件集合、夾爪、相機、是否允許重試與場景複雜度不同，不能把百分比直接當成方案排名。
- BOP 主要評估 6D pose，GraspNet 主要評估 grasp proposals；兩者都不等於完整 pick-and-place 任務成功率。
- 廠商的精度通常在指定距離、目標反射率、暖機、環境光與統計方式下量測；採購前應以自己的物件和工作距離做 proof-of-concept。
- VLA 報告多著重任務泛化，不能取代安全 PLC、碰撞檢查、速度/力限制與急停。
