# HIWIN 雙臂機械手 ROS 2 (Humble) 避障路徑規劃工作區 — 使用手冊

`~/ros2_ws/src/` 底下共 **8 個 ROS 2 package**：1 個機器人描述包
（`hiwin_description`）、1 個 MoveIt2 設定總包（`hiwin_dual_arm`），以及 6 個可插拔的
雙臂避障「規劃器」方案。各規劃器的演算法推導、參數對照表請見個規劃器的
`README.md` / `PARAMETERS.md`；

---

## 1. 這個 workspace 在做什麼

HIWIN 雙臂機械手（A 臂 RA610 + B 臂 RA605，面對面對裝、相距 1400mm）共用工作空間時，
兩臂關節軌跡可能互撞。這個 workspace 給定兩臂各自的起點/終點關節角，自動規劃出一條
**兩臂互不碰撞**的關節空間軌跡，並包成 **MoveIt2 規劃器插件**，在 `move_group` 裡當作
`planning_plugin` 使用。

---

## 2. Package 總覽

```
~/ros2_ws/src/
├── hiwin_dual_arm/                  ← 機器人描述 + MoveIt2 設定總包（非演算法）
├── hiwin_description/               ← 機器人 URDF/mesh 描述包
├── dual_arm_alm_newton_planner/     ┐
├── dual_arm_alm_cg_planner/         │  ALM 譜系（增廣拉格朗日）
├── dual_arm_alm_gd_planner/         ┘
├── dual_arm_lag_newton_planner/     ┐
├── dual_arm_lag_cg_planner/         │  純 Lagrangian 譜系
└── dual_arm_lag_gd_planner/         ┘
```

6 個規劃器解的是同一個雙臂避障問題，差別只在內層最佳化用哪種數學模型／求解方法：

| | Newton | CG（共軛梯度） | GD（最陡下降） |
|---|---|---|---|
| **ALM 模型** | `dual_arm_alm_newton_planner` | `dual_arm_alm_cg_planner` | `dual_arm_alm_gd_planner` |
| **純 Lagrangian 模型** | `dual_arm_lag_newton_planner` | `dual_arm_lag_cg_planner` | `dual_arm_lag_gd_planner` |

⚠️ ALM 與純 Lagrangian 是不同數學模型，不可混用參數。

`hiwin_dual_arm/config/dual_arm_*_planning.yaml` 這 6 個檔案各自指定
`planning_plugin` 指向對應的 `DualArmXxxPlannerManager`，並帶入該規劃器的參數；
MoveIt 依檔名（`*_planning.yaml`）自動掃描註冊成 planning pipeline，不用另外維護清單。
`ompl_planning.yaml`  OMPL 有附上官方演算法設定。

---

## 3. 環境建置與啟動（ROS 2 Humble）

### 3.1 第一次建置

```bash
# 1. 載入 ROS 2 Humble 環境
source /opt/ros/humble/setup.bash

# 2. 回到 workspace 根目錄
cd ~/ros2_ws

# 3. 編譯整個 workspace
colcon build --symlink-install

# 4. 載入這個 workspace 的環境（每開一個新終端機都要重做）
source install/setup.bash
```

> `--symlink-install` 讓 `config/*.yaml`、`launch/*.py` 用 symlink 安裝，之後**改
> yaml/launch 不用重新 build**，存檔、重啟節點就生效；但改 `src/*.cpp` 仍要重新
> `colcon build`。

### 3.2 之後每次啟動

已經建置過，就不用再跑 `colcon build`，只需重新載入環境並啟動：

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws && source install/setup.bash

# 一鍵啟動：TF + robot_state_publisher + move_group + RViz
ros2 launch hiwin_dual_arm brain.launch.py

# 只跑後端、不開 RViz
ros2 launch hiwin_dual_arm brain.launch.py use_rviz:=false
```

### 3.3 切換規劃器

RViz 左側 **MotionPlanning** 面板 → **Planning** 分頁 → **Planning Library** 下拉選單，
可選擇 `ompl`、`chomp`、`pilz_industrial_motion_planner` 或 6 個 `dual_arm_*` 之一，
選定後按 **Plan** 測試。

---

## 4. 常見問題排查

- **改了 `config/*.yaml` 但行為沒變**：確認是否用 `--symlink-install` 建置；不是的話要
  重跑 `colcon build --packages-select hiwin_dual_arm` 讓檔案複製進 `install/`。
- **選了 `dual_arm_*` pipeline，但 log 印出 `Multiple planning plugins available...
  Using 'chomp_interface/CHOMPPlanner' for now`**：代表該插件在當下環境找不到，通常是
  終端機沒有 `source install/setup.bash`（或只 source 了單一 package 的
  `local_setup.bash`），或該規劃器 package 沒編譯成功（檢查 build log 有無
  `Failed <<<` / `Aborted <<<`）。

---

## 5. 上銀（HIWIN）官方 ROS 2 Humble 程式庫

`hiwin_dual_arm` 與官方驅動程式庫不是同一個專案，但可搭配使用：

| 程式庫 | 說明 |
|---|---|
| [hiwin_ros2](https://github.com/HIWINCorporation/hiwin_ros2) | 官方 ROS 2 Humble 主力庫，提供 `ros2_control` 硬體介面與 MoveIt2 整合，支援 RA6/RS4 系列，可切模擬/實機。 |
| [hiwin_robot_client_library](https://github.com/HIWINCorporation/hiwin_robot_client_library) | 封裝與機器人控制器底層通訊的介面庫，供 `hiwin_ros2` 呼叫。 |
| [hiwin_ros](https://github.com/HIWINCorporation/hiwin_ros) | 舊版 ROS 1 程式庫，非 ROS 2。 |

若要接實體雙臂機器人（非僅 RViz/MoveIt 模擬），需另外整合官方 `hiwin_ros2` 提供的
硬體驅動。

---

## 6. 授權

各 package 內文件標示為 MIT。
