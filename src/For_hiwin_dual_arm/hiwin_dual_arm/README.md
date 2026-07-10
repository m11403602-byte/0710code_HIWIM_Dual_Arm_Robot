# hiwin_dual_arm — 雙臂機器人描述 + MoveIt2 設定總包

本包由 MoveIt Setup Assistant 產生，是 HIWIN 雙臂機械手（A 臂 `big`＝RA610、
B 臂 `small`＝RA605）的**機器人描述與 MoveIt2 設定總包**。它本身**不含避障演算法**，
角色是「底盤」：機器人模型（URDF/SRDF）、ros2_control 控制器、以及六個自訂避障
規劃器（`dual_arm_{alm,lag}_{newton,cg,gd}_planner`，見 workspace 根目錄
[README.md](../../../README.md)）要接上 MoveIt2，都是透過本包的
`config/*_planning.yaml` 與 `launch/*.py` 串接。

本文件說明：這個包裡有什麼、六個規劃器插件目前是怎麼接（以及還缺哪一步）、
和完整的啟動流程。

---

## 1. 目錄內容

```
hiwin_dual_arm/
├── package.xml / CMakeLists.txt / .setup_assistant
├── config/
│   ├── dual_hiwin.urdf.xacro / dual_hiwin.srdf   # 機器人模型 + 規劃群組
│   ├── dual_hiwin.ros2_control.xacro             # ros2_control 硬體介面
│   ├── ros2_controllers.yaml                     # controller_manager 控制器定義
│   ├── moveit_controllers.yaml                   # MoveIt ↔ 控制器 action 對應
│   ├── kinematics.yaml / joint_limits.yaml        # IK / 關節限制
│   ├── ompl_planning.yaml / pilz_cartesian_limits.yaml  # MoveIt 內建規劃管線
│   ├── dual_arm_alm_newton_planning.yaml        ┐
│   ├── dual_arm_alm_cg_planning.yaml            │
│   ├── dual_arm_alm_gd_planning.yaml            │  六個自訂避障規劃器的
│   ├── dual_arm_lag_newton_planning.yaml        │  pipeline 設定檔
│   ├── dual_arm_lag_cg_planning.yaml            │
│   ├── dual_arm_lag_gd_planning.yaml            ┘
│   ├── initial_positions.yaml / sensors_3d.yaml / moveit.rviz
│   └── moveit_controllers_copy.yaml / ros2_controllers_copy.yaml  # 備份版本
├── launch/
│   ├── rsp.launch.py                      # Robot State Publisher（算+發布 TF）
│   ├── static_virtual_joint_tfs.launch.py # 虛擬關節（world → base）靜態 TF
│   ├── spawn_controllers.launch.py        # 啟動 ros2_control 控制器
│   ├── move_group.launch.py               # MoveIt2 核心節點（含規劃管線）
│   ├── moveit_rviz.launch.py              # RViz2 視覺化
│   ├── demo.launch.py                     # 官方標準 demo（含假控制器，一鍵可視化）
│   ├── brain.launch.py / test1.launch.py  # 自訂整合啟動（見第 4 節，內容目前相同）
│   ├── warehouse_db.launch.py             # MoveIt warehouse（存規劃場景/查詢用）
│   └── setup_assistant.launch.py          # 重新打開 Setup Assistant 編輯本包
└── srcipt/joint_state_merger.py           # 合併 A/B 臂各自 joint_states 成單一 topic
```

### SRDF 規劃群組（`config/dual_hiwin.srdf`）

```
group "big"      : world -> big_tool0    (A 臂 RA610, joint 名前綴 big_joint_)
group "small"    : world -> small_tool0  (B 臂 RA605, joint 名前綴 small_joint_)
group "Dual_arm" : big + small           (雙臂合併群組 — 六個避障規劃器都是對這個 group 求解)
```

### 控制器對應（`config/moveit_controllers.yaml` + `ros2_controllers.yaml`）

機器人被拆成 **兩個獨立 namespace**（`small_arm/`、`big_arm/`），對應真實部署常見的
「兩隻手臂各自跑各自的控制器 / 甚至各自一台電腦」場景：

- `moveit_controllers.yaml`：MoveIt 要打的 action 是
  `/small_arm/joint_trajectory_controller/follow_joint_trajectory`、
  `/big_arm/joint_trajectory_controller/follow_joint_trajectory`。
- `srcipt/joint_state_merger.py`：訂閱 `/small_arm/joint_states`、`/big_arm/joint_states`，
  合併成單一 `/joint_states`（12 軸）發布，因為 MoveIt / RViz 需要看到**兩臂合併後**的
  完整關節狀態；同時把時戳改用主機時鐘蓋掉，避免因跨網段/跨機時鐘差被 MoveIt 的
  1 秒新鮮度門檻判定為過期。

> ⚠️ `config/ros2_controllers.yaml` 裡的控制器命名（`big_controller` /
> `small_controller`，無 namespace 前綴）與 `moveit_controllers.yaml`
> （`big_arm/joint_trajectory_controller` / `small_arm/joint_trajectory_controller`）
> 目前**命名不完全一致**，實際部署（尤其雙電腦分開跑 controller_manager 時）
> 請確認兩邊 namespace / 控制器名稱對得上，否則 MoveIt 執行軌跡時會找不到 action server。

---

## 2. 六個避障規劃器插件「目前」是怎麼接的

`config/` 底下已經有六份 `dual_arm_<model>_<solver>_planning.yaml`，每份都指定：

```yaml
planning_plugin: 'dual_arm_alm_newton_planner/DualArmAlmNewtonPlannerManager'   # 依檔案而異
request_adapters: >-
  default_planner_request_adapters/FixWorkspaceBounds
  ...
danger_threshold: 0.4
...（該規劃器的所有可調參數，見各 planner package 的 PARAMETERS.md）
```

這是 MoveIt2「一個 pipeline 一份 `<pipeline_id>_planning.yaml`」的標準檔名慣例
（`pipeline_id` = 檔名去掉 `_planning.yaml`，例如 `dual_arm_alm_newton_1`）。

**但目前 `package.xml` 和所有 `launch/*.py` 都還沒有真正把這六個 pipeline 接進
`move_group` 裡**——`move_group.launch.py` / `demo.launch.py` / `brain.launch.py`
呼叫 `MoveItConfigsBuilder(...).to_moveit_configs()` 時**沒有**呼叫
`.planning_pipelines(...)`，所以實際載入的規劃管線只有 MoveIt2 內建預設的
（OMPL，視 `moveit_configs_utils` 版本可能還含 Pilz），六個自訂規劃器**還沒有生效**。

要讓插件真正被 `move_group` 認得、可以在 RViz「規劃管線」下拉選單選到，需要補以下
兩步（見下一節）。

---

## 3. 如何把規劃器插件真正套用進來

### 步驟 1：`package.xml` 宣告依賴

在 `package.xml` 補上六個規劃器 package 的 `exec_depend`（讓 colcon 知道建置順序，
也讓 ament 的 plugin index 在同一個 workspace 內能被 `pluginlib` 找到）：

```xml
<exec_depend>dual_arm_alm_newton_planner</exec_depend>
<exec_depend>dual_arm_alm_cg_planner</exec_depend>
<exec_depend>dual_arm_alm_gd_planner</exec_depend>
<exec_depend>dual_arm_lag_newton_planner</exec_depend>
<exec_depend>dual_arm_lag_cg_planner</exec_depend>
<exec_depend>dual_arm_lag_gd_planner</exec_depend>
```

### 步驟 2：在 launch 檔宣告 `planning_pipelines`

修改 `move_group.launch.py`（`demo.launch.py` / `brain.launch.py` 若也要選規劃器，
一併修改），呼叫 `.planning_pipelines(...)` 把六個自訂 pipeline id 列進去：

```python
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch

def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("dual_hiwin", package_name="hiwin_dual_arm")
        .planning_pipelines(
            pipelines=[
                "ompl",                     # 保留內建 OMPL 當備援
                "dual_arm_alm_newton_1",
                "dual_arm_alm_cg_1",
                "dual_arm_alm_gd_1",
                "dual_arm_lag_newton_1",
                "dual_arm_lag_cg_1",
                "dual_arm_lag_gd_1",
            ],
            default_planning_pipeline="dual_arm_alm_newton_1",   # 預設用哪一個
        )
        .to_moveit_configs()
    )
    return generate_move_group_launch(moveit_config)
```

`pipelines` 清單中每個字串都必須對應 `config/<字串>_planning.yaml` 存在，
`MoveItConfigsBuilder` 會照這個慣例去讀對應檔案裡的 `planning_plugin` /
`request_adapters` / 各參數並注入 `move_group` node。

> 六選一同時只需要「選一個當 `default_planning_pipeline`」——六份都放進
> `pipelines` 清單，之後可在 RViz 的 MotionPlanning 面板「Planning Pipeline」
> 下拉選單即時切換，不用重啟就能比較六種演算法的規劃結果。

### 步驟 3：確認 `time_optimal` 與 `request_adapters` 搭配

六份 yaml 各自 `request_adapters` 清單已刻意**不含**
`default_planner_request_adapters/AddTimeOptimalParameterization`
（因為時間參數化改在插件的 `solve()` 內部處理，若 `time_optimal: false` 卻讓
adapter 再跑一次 TOTG，會覆蓋掉自訂的等間隔時間設定）。若要改用 `time_optimal: true`
（TOTG），兩種做法皆可、擇一：
- 維持插件內建 TOTG（不用動 `request_adapters`），或
- 改回讓 adapter 做 TOTG，並把該 yaml 的 `time_optimal` 相關邏輯停用。
  一般建議維持現狀（插件內處理）即可，不需疊加兩次。

### 步驟 4：重新編譯

```bash
colcon build --symlink-install --packages-up-to hiwin_dual_arm
source install/setup.bash
```

（`--packages-up-to hiwin_dual_arm` 會連同六個規劃器 package 一起建置，
因為步驟 1 已在 `package.xml` 宣告了依賴。）

---

## 4. 完整啟動流程

### 4.1 前置：進入 ROS 2 Humble 環境並編譯（distrobox 容器）

```bash
distrobox enter hiwin-humble-env

# 容器內先清掉可能殘留的系統 ROS 環境變數，避免跟 humble 打架
unset ROS_VERSION ROS_ROOT AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH
source /opt/ros/humble/setup.bash

cd ~/0706_hiwim         # workspace 根目錄
rm -rf build/ install/ log/     # 乾淨重建（有殘留快取才需要）
colcon build --symlink-install --cmake-args -DCMAKE_PREFIX_PATH=/opt/ros/humble
source install/setup.bash

export ROS_DOMAIN_ID=24   # 若多機/多容器共用網路，統一網域 ID
```

### 4.2 只要看軌跡有沒有異常（純可視化，最快）

```bash
ros2 launch hiwin_dual_arm demo.launch.py
```

`demo.launch.py` 用 `moveit_configs_utils` 內建的 `generate_demo_launch`，會自動
一次啟動：虛擬 TF、`robot_state_publisher`、假控制器（fake hardware）、
`move_group`、RViz2。適合純規劃/可視化除錯，不需真實硬體或 `spawn_controllers`。

> 若已依第 3 節接好六個自訂 pipeline，RViz 的 MotionPlanning 面板即可切換規劃器；
> 若尚未接（目前預設狀態），這裡只能用 MoveIt 內建的 OMPL。

### 4.3 分件手動啟動（對應真實機器人 / 除錯用，逐一啟動每個節點）

依序在各自終端機（或同一個 launch 檔用 `IncludeLaunchDescription` 串起來，
即 `brain.launch.py` 的做法）：

```bash
# 1) 虛擬關節 TF（world -> base）
ros2 launch hiwin_dual_arm static_virtual_joint_tfs.launch.py

# 2) Robot State Publisher（算 12 軸 TF 樹）
ros2 launch hiwin_dual_arm rsp.launch.py

# 3) 啟動 ros2_control 控制器（真實/模擬硬體介面 + controller_manager）
ros2 launch hiwin_dual_arm spawn_controllers.launch.py

# 4)（若為真實雙臂分開兩個 namespace 部署）合併 joint_states
ros2 run hiwin_dual_arm joint_state_merger.py
#   訂閱 /small_arm/joint_states + /big_arm/joint_states -> 發布合併後 /joint_states

# 5) MoveGroup（MoveIt 核心大腦：接收規劃請求、跑避障演算法、送出軌跡）
ros2 launch hiwin_dual_arm move_group.launch.py

# 6) RViz2 視覺化操作介面
ros2 launch hiwin_dual_arm moveit_rviz.launch.py
```

### 4.4 一鍵整合啟動：`brain.launch.py`

`brain.launch.py`（與內容相同的 `test1.launch.py`）把「虛擬 TF → RSP → MoveGroup
→ RViz（可選）」串成一個 launch 檔：

```bash
ros2 launch hiwin_dual_arm brain.launch.py            # 含 RViz
ros2 launch hiwin_dual_arm brain.launch.py use_rviz:=false   # 不開 RViz（純背景跑大腦）
```

> ⚠️ 注意：`brain.launch.py` 目前**沒有**包含 `spawn_controllers.launch.py`，
> 也沒有 `joint_state_merger`。純規劃/可視化沒問題，但若要讓軌跡真的送去
> 執行（`FollowJointTrajectory` action），需要另外啟動
> `spawn_controllers.launch.py`（與/或真實控制器節點）+ 視部署方式決定是否需要
> `joint_state_merger`。若要做成單一入口，可仿照第 3 節的模式，在
> `brain.launch.py` 內用 `IncludeLaunchDescription` 加入
> `spawn_controllers.launch.py` 與一個 `Node(package="hiwin_dual_arm",
> executable="joint_state_merger.py")`（需先在 `CMakeLists.txt` / `package.xml`
> 補上 Python 執行檔安裝設定，目前 `srcipt/joint_state_merger.py` 尚未被
> `install(PROGRAMS ...)` 安裝為可執行的 ROS 節點）。

### 4.5 其他輔助 launch

```bash
ros2 launch hiwin_dual_arm warehouse_db.launch.py     # 啟動場景/查詢資料庫（規劃場景儲存用）
ros2 launch hiwin_dual_arm setup_assistant.launch.py  # 重新打開 Setup Assistant 編輯本包
```

---

## 5. 驗證插件有沒有真正接上

```bash
# 1. 確認六個規劃器 package 都已建置且 plugin xml 有被 ament_index 收錄
ros2 pkg prefix dual_arm_alm_newton_planner

# 2. move_group 啟動時觀察 log，會印出目前載入的 planning pipeline 清單
ros2 launch hiwin_dual_arm move_group.launch.py --show-args   # 檢查參數
# 啟動後看 move_group 節點的 log，搜尋 "Using planning pipeline" / pipeline 名稱

# 3. RViz MotionPlanning 面板 -> Planning Pipeline 下拉選單
#    應能看到 ompl 之外，六個 dual_arm_* 選項（前提：已完成第 3 節的兩步修改）

# 4. 動態切換某個規劃器參數（免重啟 move_group）
ros2 param set /move_group dual_arm_alm_newton_1.danger_threshold 0.35
```

---

## 6. 換機器人 / 換場景時要同步調整的地方

- `config/dual_hiwin.urdf.xacro`、`dual_hiwin.srdf`：機器人本體、group、
  自我碰撞矩陣。
- `config/kinematics.yaml`、`joint_limits.yaml`：IK 與速度/加速度限制
  （六個規劃器的 `time_optimal: true` TOTG 需要這裡的加速度限制，否則用預設
  `1 rad/s²` 並印警告）。
- `config/moveit_controllers.yaml`、`ros2_controllers.yaml`：控制器/命名空間、
  action 名稱要對得上實機。
- 六個 `dual_arm_*_planning.yaml` 裡的 `joint_prefix_A` / `joint_prefix_B`：
  要跟這裡 SRDF 的關節命名一致（目前為 `big_joint_` / `small_joint_`）。
- 六個規劃器 package 內寫死的機器人幾何（包覆球座標半徑、FK、底座位置）：
  不在本包內，需個別去改該 planner package 的 `newton_solver.cpp` /
  `cg_solver.cpp` / `gd_solver.cpp`，詳見各 package 的 `PARAMETERS.md`
  第八節「機器人幾何」。

---

## 7. 授權

沿用 MoveIt2 產生的預設授權：BSD。
