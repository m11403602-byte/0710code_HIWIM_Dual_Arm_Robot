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

所有原始碼（核心演算法 + MoveIt 插件介面）都編進**單一 .so**（例如
`libdual_arm_alm_newton_planner_1.so`），刻意不拆成獨立核心庫 + 插件庫兩個 .so —— 
因為含 Eigen 矩陣的物件跨 .so 邊界傳遞，容易和 MoveIt 的記憶體對齊假設衝突而崩潰。
同理，編譯選項固定 `-O3`、**不可加 `-march=native`**（會讓 Eigen 用 32-byte AVX 對齊，
與標準編譯的 MoveIt .so 對齊不一致，於軌跡物件析構時 segfault）。

---



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



## 11. 授權

各 package 內文件標示為 MIT。
