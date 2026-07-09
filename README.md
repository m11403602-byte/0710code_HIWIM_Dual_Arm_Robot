# HIWIN 雙臂機械手 ROS 2 (Humble) 避障路徑規劃工作區

本 README 說明整個 workspace（`src/0710_code/`）底下 **8 個 ROS 2 package** 彼此的關係，
以及各個「規劃器（planner）方案」在解什麼問題、差異在哪。內容整理自各 package 自己的
`README.md` / `PARAMETERS.md`，供快速掌握全貌用；細節請仍以各 package 內文件為準。

---

## 1. 這個 workspace 在做什麼

HIWIN 雙臂機械手（A 臂 RA610 + B 臂 RA605，面對面對裝、相距 1400mm）在共用工作空間內
動作時，兩隻手臂的關節軌跡可能互相撞到。這個 workspace 的目標是：

> 給定兩臂各自的起點關節角與終點關節角，自動規劃出一條**兩臂互不碰撞**的關節空間軌跡，
> 並包成 **MoveIt2 規劃器插件**，可直接在 `move_group` 裡當作 `planning_plugin` 使用。

---

## 2. Package 總覽

```
src/0710_code/
├── hiwin_dual_arm/                  ← 機器人描述 + MoveIt2 設定總包（非演算法）
├── hiwin_description/               ← 機器人 URDF/mesh 描述包（被 hiwin_dual_arm 引用）
├── dual_arm_alm_newton_planner/     ┐
├── dual_arm_alm_cg_planner/         │  ALM 譜系（增廣拉格朗日）
├── dual_arm_alm_gd_planner/         ┘
├── dual_arm_lag_newton_planner/     ┐
├── dual_arm_lag_cg_planner/         │  純 Lagrangian 譜系
└── dual_arm_lag_gd_planner/         ┘
```

也就是說：**2 個機器人/MoveIt 設定包 + 6 個「規劃器（方案）」**。
這 6 個規劃器全部在解「同一個雙臂避障問題」，差別只在**內層最佳化演算法**用哪一種
數學模型／哪一種求解方法 —— 可視為 2（數學模型）× 3（求解方法）的組合矩陣：

| | Newton（二階，含 Hessian） | CG（共軛梯度） | GD（最陡下降） |
|---|---|---|---|
| **ALM 模型**（增廣拉格朗日） | `dual_arm_alm_newton_planner` | `dual_arm_alm_cg_planner` | `dual_arm_alm_gd_planner` |
| **純 Lagrangian 模型** | `dual_arm_lag_newton_planner` | `dual_arm_lag_cg_planner` | `dual_arm_lag_gd_planner` |

⚠️ 兩個譜系（ALM vs 純 Lagrangian）是**不同的數學模型**，各自 package 的
README 都特別註明「不可混用」。

---

## 3. `hiwin_dual_arm` — 機器人本體與 MoveIt2 設定包

由 MoveIt Setup Assistant 產生的標準設定包，本身**不含避障演算法**，作用是把上面 6 個
規劃器串接到實際機器人：

- `config/dual_hiwin.urdf.xacro`、`dual_hiwin.srdf`：雙臂機器人模型與規劃群組（planning
  group）、自我碰撞矩陣定義。
- `config/dual_hiwin.ros2_control.xacro`、`ros2_controllers.yaml`：ros2_control 硬體介面
  與控制器設定。
- `config/dual_arm_{alm,lag}_{newton,cg,gd}_planning.yaml`：**六個規劃器各自對應的
  planning pipeline 設定檔**，每個檔案指定 `planning_plugin` 指向對應的
  `DualArmXxxPlannerManager`，並帶入該 package 的參數。這些檔案是被
  `MoveItConfigsBuilder` 依檔名規則（`config/*_planning.yaml`）**自動掃描註冊**成
  planning pipeline 的，不需要另外維護一份 pipeline 清單。
- `config/kinematics.yaml`、`joint_limits.yaml`、`ompl_planning.yaml`、
  `pilz_cartesian_limits.yaml`：MoveIt2 標準設定（IK solver、關節限制、備用 OMPL/Pilz
  規劃管線；`ompl_planning.yaml` 只放 OMPL 官方演算法設定，不要混入自訂規劃器參數）。
- `launch/`：`brain.launch.py`（**整合啟動入口**，串起 TF、robot_state_publisher、
  move_group、RViz）、`demo.launch.py`（rviz demo）、`move_group.launch.py`、
  `moveit_rviz.launch.py`、`rsp.launch.py`、`static_virtual_joint_tfs.launch.py`、
  `spawn_controllers.launch.py` 等啟動檔。
- `srcipt/joint_state_merger.py`：把 A/B 兩臂各自的 `joint_states` 合併成單一 topic，供
  雙臂同時顯示/控制。

換句話說，`hiwin_dual_arm` 是「底盤」，6 個 `dual_arm_*_planner` 是可插拔的「規劃大腦」，
換掉 `planning_plugin` 指向哪個 package 就等於換演算法，機器人模型與控制器不用動。

---

## 上銀（HIWIN）官方 ROS 2 Humble 程式庫

本 workspace 中的 `hiwin_dual_arm`（機器人描述 + MoveIt2 設定）與上銀官方發布的 ROS 2
驅動程式庫並非同一個專案，但屬於同一生態系、可互相搭配使用。上銀官方 GitHub 組織
（[HIWINCorporation](https://github.com/HIWINCorporation)）目前維護以下與 ROS 2 Humble
相關的程式庫：

| 程式庫 | 連結 | 說明 |
|---|---|---|
| **hiwin_ros2** | [github.com/HIWINCorporation/hiwin_ros2](https://github.com/HIWINCorporation/hiwin_ros2) | 官方 ROS 2 Humble 主力程式庫。提供 `ros2_control` 硬體介面（可對實體機器人做即時控制/監控）與 MoveIt2 整合（運動規劃、軌跡執行），底層基於 `hiwin_robot_client_library`，支援 RA6 / RS4 系列機器人，可切換模擬或實機連線。 |
| **hiwin_robot_client_library** | [github.com/HIWINCorporation/hiwin_robot_client_library](https://github.com/HIWINCorporation/hiwin_robot_client_library) | 提供給開發者的簡化介面函式庫，封裝與 HIWIN 機器人控制器的底層通訊，供上層（如 `hiwin_ros2`）呼叫。 |
| **hiwin_ros** | [github.com/HIWINCorporation/hiwin_ros](https://github.com/HIWINCorporation/hiwin_ros) | 舊版 ROS 1（含 Windows ROS）程式庫，用於控制/監控 HIWIN 機器人與電動夾爪，非 ROS 2 版本。 |

> 本 workspace 的 6 個避障規劃器（`dual_arm_*_planner_1`）是自行開發的 MoveIt2
> `planning_plugin`，用來取代/搭配官方 `hiwin_ros2` 中預設的 MoveIt2 規劃管線；
> 若要接實體 HIWIN 雙臂機器人（而非僅在 rviz/MoveIt 中模擬），需另外整合官方
> `hiwin_ros2`（或 `hiwin_robot_client_library`）提供的硬體驅動與通訊介面。

---

## 4. 共同架構：雙層最佳化

六個規劃器共用**同一套外層框架**，差別在內層：

```
外層 (avoidance_system)：
  生成初始軌跡 → 碰撞偵測 → 找出危險段（5 點）→ 呼叫內層優化 →
  Spline 重建局部軌跡 → 重新檢查（最多 15 輪，可調）

內層（依 package 而異）：
  以「平滑危險因子」 sj = exp( ln(0.5) / (Ri+Rj)² · d² ) 為避碰約束，
  對危險段的中間路徑點做最佳化，讓兩臂各自的包覆球（bubble）彼此不重疊
```

- 兩臂用「包覆球模型」近似：A 臂（RA610）16 顆球（4 底盤 + 12 手臂），
  B 臂（RA605）18 顆球（8 底盤 + 10 手臂）。
- 危險因子閾值 `danger_threshold = 0.4`，實際判碰用 `0.4 + collision_tolerance(0.1) = 0.5`
  留緩衝帶。
- 每個 package 都提供：MoveIt2 插件（`planner_manager`）、可獨立執行的
  `run_standalone`（餵 waypoints、跑避障、匯出 CSV，方便除錯/做實驗數據）、
  以及一份 `PARAMETERS.md`（列出所有可調參數、預設值、對應 MATLAB 原型變數）。

---

## 5. 兩種數學模型的差異：ALM vs 純 Lagrangian

### ALM 譜系（`dual_arm_alm_*`）

- 對應 MATLAB `Dual_Arm_Inequality_ALM_{Newton,CG,GD}_v6/v7`。
- 用 **PHR 增廣拉格朗日法（Augmented Lagrangian Method）**：外層更新罰參數 `c` 與
  乘子 `μ`，內層針對目前罰函數做無約束最佳化。
- 決策變數只有路徑點 `X`（乘子 `μ`、罰參數 `c` 是外層迭代更新的**參數**，不是決策變數）。
- 收斂判準：KKT 三條件同時滿足 —— 可行性 `v_pure ≤ eps_v`、梯度
  `‖grad L‖ ≤ eps_g`、互補性 `compl ≤ eps_compl`。

### 純 Lagrangian 譜系（`dual_arm_lag_*`）

- 對應 MATLAB `Dual_Arm_Lagrangian_{Newton,CG,Gradient}_v2`。
- 用 **Slack 變數 + Lagrangian**（Bertsekas §3.3.2）把不等式約束 `D_i(X) ≤ θ` 轉成等式：
  `h_i = D_i(X) − θ + s_i² = 0`。
- 決策變數擴增為 **V = [X; λ; S]**（路徑點、乘子、slack 全部一起優化，
  不是外層更新），維度達 1116。
- 梯度是完整 KKT 一階殘差 `G = [G_X; G_λ; G_S]`，一次性對整個 V 求解/下降，
  沒有「外層更新乘子、內層優化 X」的兩層結構。

兩者是根本不同的建模方式（ALM 是「外層調參數／內層優化」的雙層結構；
Lagrangian 版把乘子也塞進決策變數、單層求解到 KKT），因此不可混用、也不能共用參數。

---

## 6. 三種內層求解方法（Newton / CG / GD）

在 ALM 或 Lagrangian 模型底下，各自可再選擇不同的無約束最佳化求解器：

| 方法 | 檔案 | 特性 |
|---|---|---|
| **Newton** | `newton_solver.cpp` | 用完整 Hessian（`H_smooth + H_collision`，中心差分求得），`d = -(H\G)`，Eigen `LDLT` 分解求下降方向，收斂快但每步成本高（需組 Hessian） |
| **CG（共軛梯度）** | `cg_solver.cpp` | Fletcher-Reeves 共軛梯度，只需梯度、不用 Hessian，配合 1D Newton 線搜索決定步長，記憶體/計算成本較低 |
| **GD（最陡下降）** | `gd_solver.cpp` | 最簡單的梯度下降 + 1D Newton 線搜索，實作最簡單、但收斂通常最慢 |

也就是「用不用二階資訊、用多少」的取捨：Newton 最精確但貴、GD 最便宜但慢、CG 介於中間。

---

## 7. 各 package 的組成（以任一 planner 為例，其餘 5 個結構相同）

```
dual_arm_<model>_<solver>_planner_1/
├── CMakeLists.txt / package.xml / <pkg>.xml   # ament + MoveIt plugin 描述
├── config/<pkg>.yaml                          # 規劃器可調參數（yaml，免重編）
├── include/<pkg>/
│   ├── {newton,cg,gd}_solver.hpp              # 第1層：內層求解器
│   ├── avoidance_system.hpp                   # 第2層：外層碰撞修復迴圈
│   ├── data_io.hpp                            # CSV 匯出工具
│   └── planner_manager.hpp                    # 第3層：MoveIt2 PlannerManager 介面
├── src/                                       # 對應上面各 .hpp 的實作
├── standalone/run_standalone.cpp              # 獨立執行檔（不經 MoveIt，跑避障+匯出CSV）
├── README.md                                  # 該演算法的詳細說明
└── PARAMETERS.md                              # 參數對照表（yaml可調 vs 寫死）
```

所有原始碼（核心演算法 + MoveIt 插件介面）都編進**單一 .so**（例如
`libdual_arm_alm_newton_planner_1.so`），刻意不拆成獨立核心庫 + 插件庫兩個 .so —— 
因為含 Eigen 矩陣的物件跨 .so 邊界傳遞，容易和 MoveIt 的記憶體對齊假設衝突而崩潰。
同理，編譯選項固定 `-O3`、**不可加 `-march=native`**（會讓 Eigen 用 32-byte AVX 對齊，
與標準編譯的 MoveIt .so 對齊不一致，於軌跡物件析構時 segfault）。

---

## 8. 常用可調參數（六個 package 共通，寫在各自 yaml，免重編即可生效）

| 參數 | 說明 |
|---|---|
| `danger_threshold` | 危險因子閾值（避障鬆緊） |
| `collision_tolerance` | 碰撞判定緩衝帶 |
| `path_weight` | A/B 臂成本權重 |
| `max_refinement_iter` | 外層修復最多輪數 |
| `smooth_w` / `smooth_w_H` / `smooth_w_T` | 軌跡平滑度權重 |
| `joint_prefix_A` / `joint_prefix_B` | 對應 SRDF 的關節名前綴，換機器人時要改 |
| `time_optimal` | true=TOTG 時間最佳化；false=自訂等間隔（`path_total_time`/`min_time_interval`）|
| `export_csv_prefix` / `export_level` | 是否及多詳細匯出規劃過程 CSV（除錯/論文數據用）|

機器人幾何（包覆球座標半徑、FK、底座位置）、ALM/CG/GD 內部數值容忍度等屬於「寫死於
原始碼、需重編」的參數，詳見各 package 的 `PARAMETERS.md`。

---

## 9. 如何選規劃器 / 如何使用

1. 在 `move_group` 的 planning pipeline 設定裡，把 `planning_plugin` 指向想用的
   package，例如：
   ```yaml
   planning_plugin: dual_arm_alm_newton_planner_1/DualArmAlmNewtonPlannerManager
   ```
   對應 6 選 1，或用 `hiwin_dual_arm/config/dual_arm_<model>_<solver>_1_planning.yaml`
   直接載入。
2. 或不經 MoveIt，直接跑該 package 的 `run_standalone` 餵 waypoints 做離線測試/畫圖用
   資料匯出。
3. 各 package 內的 `README.md` 有更完整的演算法推導、CSV 匯出格式、與 MATLAB 原型的
   對應差異、已知踩雷筆記；`PARAMETERS.md` 有逐項參數表。若要換機器人（非 RA610/RA605
   雙臂），需同步調整六個 package 的機器人幾何常數與 `hiwin_dual_arm` 的 URDF/SRDF。

---

## 10. 環境安裝與啟動教學（ROS 2 Humble + distrobox）

本機的 ROS 2 Humble 是跑在 **distrobox 容器**（`hiwin-humble-env`）裡，而不是直接裝在
host OS 上，使用前務必先進容器、並清掉 host 殘留的 ROS 環境變數，否則會混到別的 ROS
distro（例如 host 上若另外裝了 Jazzy，`colcon build` 會抓錯 `rosidl_typesupport` 版本而
編譯失敗）。

### 10.1 首次 / 乾淨重建

```bash
# 1. 進入 Humble 容器
distrobox enter hiwin-humble-env

# 2. 清掉 host 可能殘留、會干擾 distro 判斷的環境變數
unset ROS_VERSION ROS_ROOT AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH

# 3. 載入 Humble 環境
source /opt/ros/humble/setup.bash

# 4. 回到 workspace 根目錄，清掉舊的編譯產物（非必要，但遇到怪問題時的第一招）
cd ~/0710_code
rm -rf build/ install/ log/

# 5. 編譯整個 workspace（含 6 個規劃器 + hiwin_dual_arm/hiwin_description）
colcon build --symlink-install

# 6. 載入這個 workspace 自己的環境（每開一個新終端機都要重做這步）
source install/setup.bash
```

> `--symlink-install` 讓 `config/*.yaml`、`launch/*.py` 用 symlink 安裝到 `install/`，
> 之後**改 yaml/launch 檔不用重新 colcon build**，改完存檔、重啟對應節點就生效；但
> C++ 原始碼（`src/*.cpp`）修改後仍然要重新 `colcon build`。若沒加
> `--symlink-install`（純 copy 安裝），任何 `src/` 底下的修改都要重跑 `colcon build`
> 才會反映到 `install/`，這是「明明改了設定，行為卻沒變」最常見的原因。

### 10.2 日常啟動（只改過 yaml，不用重建的情況）

```bash
distrobox enter hiwin-humble-env
source /opt/ros/humble/setup.bash
cd ~/0710_code && source install/setup.bash

# 統一網域 ID（多人共用網路時避免互相干擾/搶 topic）
export ROS_DOMAIN_ID=24

# 一鍵啟動：TF + robot_state_publisher + move_group + RViz
ros2 launch hiwin_dual_arm brain.launch.py

# 只想跑後端、不開 RViz（例如純程式互動、或另開視窗自己啟動 RViz）
ros2 launch hiwin_dual_arm brain.launch.py use_rviz:=false
```

### 10.3 在 RViz 裡切換規劃器

RViz 開啟後於左側 **MotionPlanning** 面板 → **Planning** 分頁 → **Planning Library**
下拉選單，會列出所有已註冊的 planning pipeline（`ompl`、`chomp`、
`pilz_industrial_motion_planner`，以及本專案的 6 個 `dual_arm_*`）。選定 pipeline 後即可
按 **Plan** 測試對應演算法規劃出的軌跡；下方 **Planner** 欄若該 pipeline 只有單一插件則
不需另選。

### 10.4 常見問題排查

- **改了 `config/*.yaml` 但行為沒變**：確認是否用 `--symlink-install` 編譯；若不是，
  要重跑 `colcon build --packages-select hiwin_dual_arm`（純設定檔不需要編譯 C++，只是
  需要 colcon 重新把檔案複製進 `install/`）。
- **RViz 選了某個 `dual_arm_*` pipeline，log 卻印出
  `Multiple planning plugins available... Using 'chomp_interface/CHOMPPlanner' for now`**：
  代表該 pipeline 對應的 `planning_plugin` 沒有被實際載入（yaml 設定沒問題但插件
  在執行當下的環境裡找不到），九成是**目前終端機沒有正確 source 到該規劃器 package
  的環境**——確認是對 `install/setup.bash`（會匯總所有 package）source，而不是只
  source 了單一 package 的 `local_setup.bash`，並確認該次 `colcon build` 有把 6 個
  `dual_arm_*_planner` 都成功編完（看 build log 有沒有 `Failed <<<` / `Aborted <<<`）。
- **一直抓錯 ROS distro / 編譯出現 `rosidl_typesupport_c` 找不到之類的 CMake 錯誤**：
  通常是進容器前忘記 `unset` host 殘留的 `AMENT_PREFIX_PATH`/`CMAKE_PREFIX_PATH`，
  或根本沒進 `distrobox enter hiwin-humble-env` 就直接跑指令，導致 `colcon` 抓到 host
  上別的 ROS distro 的工具鏈。
- **多台機器/多個終端機互相干擾（收到不該收到的 topic、服務衝突）**：檢查
  `ROS_DOMAIN_ID` 是否每個終端機都設成一樣的值（`echo $ROS_DOMAIN_ID` 確認）。

---

## 11. 授權

各 package 內文件標示為 MIT。
