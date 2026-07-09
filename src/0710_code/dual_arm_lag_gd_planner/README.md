# dual_arm_lag_gd_planner

雙臂避障路徑規劃器 — ROS 2 Humble / MoveIt2 插件。

使用 C++17 + Eigen3 實作。內層採用 **純 Lagrangian（決策變數 V=[X;λ;S]）+ GD（最陡下降）+ 1D Newton 線搜索**。

---

## 演算法概觀

兩隻機械臂（RA610 16 球 / RA605 18 球包覆模型），給定起點與終點關節角，求一條兩臂互不碰撞的關節空間軌跡。

採雙層最佳化：

```
外層 (avoidance_system):  生成初始軌跡 -> 碰撞偵測 -> 找危險段 5 點 ->
                          呼叫內層優化 -> Spline 重建 -> 重新檢查 (最多 max_refinement_iter 輪，可調參數)
內層 (gd_solver):         純 Lagrangian 單迴圈 (V=[X;λ;S] ∈ ℝ^1116) + GD + 線搜索
```

危險因子閾值 `danger_threshold = 0.35`、碰撞判定餘隙 `collision_tolerance = 0.15`（皆 yaml 預設，須為正值）。碰撞邊界(0.5) = `danger_threshold + collision_tolerance`；**設定時兩者之和不得超過碰撞邊界(0.5)**（0.5 = 兩球相切，超過即會漏判真碰撞）。

---

## 架構：單一 .so

所有原始碼（核心演算法 + MoveIt 插件介面）編進**單一共享庫** `libdual_arm_lag_gd_planner.so`。

| 組成 | 角色 |
|------|------|
| `gd_solver` | 第 1 層: 內層 GD 求解器（V=[X;λ;S]）+ FK + 包覆球 + 危險因子 |
| `avoidance_system` | 第 2 層: 外層碰撞修復迴圈 + Spline 重建 |
| `data_io` | CSV 寫入工具（若需要查看關節路徑與危險因子變化） |
| `planner_manager` | 第 3 層: MoveIt2 PlannerManager / PlanningContext |

---

## 優化流程與變數說明

### 外層：碰撞修復迴圈 (`AvoidanceSystem::run_optimization`)

```
┌────────────────────────────────────────────────────────────────────┐
│ Clamped Cubic Spline 生成初始軌跡 (A/B 臂 起點 → 終點)                │
│        │                                                            │
│        ▼                                                            │
│ 逐步計算危險因子 D_m = calc_df(...) → max_D(t)                       │
│        │                                                            │
│        ▼                                                            │
│ max_D(t) >= 碰撞邊界(0.5) (=danger_threshold + collision_tolerance)│
│ 代表碰撞持續優化 ?  ──否──▶ 完成 (has_collision=false)               │
│        │是                                                          │
│        ▼                                                            │
│ find_collision_targets(): 取危險段 5 個控制點                        │
│   [Head, q1, peak, q3, Tail]                                        │
│        │                                                            │
│        ▼                                                            │
│ 取 3 個內部點 (q1, peak, q3) 組成 X (36 維), 連同 λ0/S0 組 V0        │
│ 呼叫內層優化器 run_lag(V0) → V* = [X*; λ*; S*]                       │
│        │                                                            │
│        ▼                                                            │
│ regenerate_trajectory_global(): 用 X* 做局部 Spline 重建整段軌跡      │
│        │                                                            │
│        ▼                                                            │
│ 重新計算 max_D(t)，回到「危險判定」 (最多 max_refinement_iter 輪)     │
└────────────────────────────────────────────────────────────────────┘
```


### 內層：純 Lagrangian 梯度下降求解器 (`GdSolver::run_lag`)

> 圖中「參數名=數值」為 yaml 預設值示例，實際以 config 為準。

```
┌──────────────────────────────────────────────────────────────────┐
│ 初始化: V = [X0; λ = lag_lam0(=30); S = lag_s0(=1)]                │
│        │                                                          │
│        ▼                                                          │
│  ┌── 主迴圈 (最多 lag_max_iter=500 步) ──────────────────────────┐  │
│  │  計算 KKT 殘差 G(V) = [G_X; G_λ; G_S]                          │  │
│  │  d = -G   (最陡下降方向)                                       │  │
│  │  1D Newton 線搜索求步長 alpha                                   │  │
│  │  V ← V + alpha·d                                               │  │
│  │        ▼                                                       │  │
│  │  收斂判定: max_D ≤ danger_threshold + lag_tol_phys_margin(=0.01)│ │
│  │           && |Δmax_D| ≤ lag_tol_stable(=0.01)                  │  │
│  │           (stationarity 刻意停用 — KKT 鞍點下 ‖G‖ 不收斂)        │  │
│  │      ──否──▶ 繼續下一步                                         │  │
│  └── 是 ──▶ 回傳 V* = [X*; λ*; S*]、SolverLog ──────────────────┘  │
└──────────────────────────────────────────────────────────────────┘
```

### 主要變數

| 變數 | 意義 | 維度 |
|------|------|------|
| `V` | 決策變數 `[X; λ; S]`（一次解整包，λ/S 非外層更新）| 1116 |
| `X` | 危險段 3 個內部控制點 × 雙臂各 6 軸（degree）| 36 |
| `λ` | 不等式約束乘子（決策變數之一）| `num_C_` |
| `S` | 鬆弛變數（以 S² 把不等式轉等式）| `num_C_` |
| `G` | Lagrangian 對 V 的梯度（KKT 殘差 `[G_X; G_λ; G_S]`）| 1116 |
| `d` | 最陡下降方向 `-G` | 1116 |

各參數的預設值與說明詳見 [PARAMETERS.md](PARAMETERS.md)。

---

## 目錄結構

- **根目錄檔案**（`CMakeLists.txt` / `package.xml` / `*.xml` / `README.md` / `PARAMETERS.md`）：**建置與描述層**——定義怎麼編譯和對外說明文件。
- **`config/`**：**執行期參數層**——存放 move_group 啟動時讀入的 yaml 參數，需複製為 `hiwin_dual_arm/config/dual_arm_lag_gd_planning.yaml`。
- **`include/dual_arm_lag_gd_planner/`**：**介面宣告層**。
- **`src/`**：**演算法實作層**。

```
dual_arm_lag_gd_planner/
├── CMakeLists.txt
├── package.xml
├── dual_arm_lag_gd_planner.xml
├── README.md
├── PARAMETERS.md
├── config/
│   └── dual_arm_lag_gd_planner.yaml
├── include/dual_arm_lag_gd_planner/
│   ├── gd_solver.hpp
│   ├── avoidance_system.hpp
│   ├── data_io.hpp
│   └── planner_manager.hpp
└── src/
    ├── gd_solver.cpp
    ├── avoidance_system.cpp
    ├── data_io.cpp
    └── planner_manager.cpp
```

| 檔案 | 功能 |
|------|------|
| `CMakeLists.txt` | 建置設定：編譯單一 .so |
| `package.xml` | ROS 2 套件描述與相依宣告 |
| `dual_arm_lag_gd_planner.xml` | pluginlib 插件描述（讓 MoveIt2 動態載入） |
| `README.md` | 本文件 |
| `PARAMETERS.md` | 參數對照表（可調參數） |
| `config/dual_arm_lag_gd_planner.yaml` | 規劃器參數（需複製為 `hiwin_dual_arm/config/dual_arm_lag_gd_planning.yaml`） |
| `gd_solver.hpp/.cpp` | 第 1 層：FK、包覆球、危險因子、純 Lagrangian GD 求解 |
| `avoidance_system.hpp/.cpp` | 第 2 層：外層碰撞修復迴圈、Spline 重建、CSV 匯出 |
| `data_io.hpp/.cpp` | CSV 寫入工具 |
| `planner_manager.hpp/.cpp` | 第 3 層：MoveIt2 插件介面（radian↔degree、時間參數化） |

---

## 編譯

放進 ROS 2 workspace 的 `src/` 後：

```bash
cd ~/ros2_ws
colcon build --packages-select dual_arm_lag_gd_planner
source install/setup.bash
```

需求：ROS 2 Humble、MoveIt2、Eigen3、C++17 編譯器。

### ⚠️ 編譯選項：使用 `-O3`，**不要用 `-march=native`**

CMake 預設 Release `-O3 -DNDEBUG`（**刻意不含 `-march=native`**，會與 MoveIt 的 Eigen 記憶體對齊衝突而崩潰）。

---

## 使用方式（MoveIt2 插件）

> 前提：與 `hiwin_dual_arm`（機器人描述 + MoveIt 設定）同 workspace。

1. 把本包 `config/dual_arm_lag_gd_planner.yaml` 複製為
   `hiwin_dual_arm/config/dual_arm_lag_gd_planning.yaml`（檔名依規劃器而不同）。
   檔內已含 `planning_plugin` 與 `request_adapters` 完整設定，複製即可、無需再改 pipeline。
2. 啟動（例）：

```bash
ros2 launch hiwin_dual_arm brain.launch.py
```

規劃請求需含 joint goal constraints。

**改參數**：編輯 yaml → 重新複製到 `hiwin_dual_arm/config/` → 重啟 move_group 生效（**不用重編**）。

---

## CSV 匯出

對外唯一入口：`export_unified(prefix, level)`。輸出到 `prefix/<timestamp>_GD/` 目錄（每次匯出自成一個時間戳資料夾，不互相覆蓋）。

| level | 檔案 | 內容 |
|:---:|------|------|
| **0** | （無） | 完全不匯出（總開關；yaml 目前預設） |
| **1** | `meta.csv` | 參數快照（key/value） |
| | `summary.csv` | 每輪修復計分板 |
| | `inner.csv` | 內層逐步串接（iter / ‖G‖ / ‖d‖ / alpha / λ / S） |
| | `danger_final.csv` | 初始 vs 最終每步 MaxD 對照 |
| | `danger_rounds.csv` | 長表 round／step／MaxD_in／MaxD_out |
| | `targets.csv` | 每輪 5 特徵點（索引 + D 值 + 優化前後關節角） |
| **2** | 以上 6 檔 + `constraints_all.csv` | 全約束 D 值大表 |
| | `path_original.csv` | 初始軌跡全點 |
| | `path_evolution.csv` | 長表 round／step／A1..B6 |

MoveIt 路徑由 yaml 的 `export_csv_prefix`（預設 `./lag_data`）與 `export_level` 控制（預設 0 = 不匯出；要留資料改 1 或 2 後重啟 move_group）。所有 CSV 可用 Excel、Python `pandas.read_csv` 直接開啟。

---

## 需要調整的地方

部署前請依實際機器人調整：

1. **關節名**（`src/planner_manager.cpp` 的 `solve()`）：目前用 A 臂 `big_joint_1~6`、B 臂 `small_joint_1~6`，改成你 SRDF 的實際關節名。
2. **planning group**：solve 用 `req.group_name`，確認 SRDF 的 group 設定。
3. **機器人底座**（`avoidance_system.cpp` 建構函數）：目前 A 臂 `Ty(700)Rz(180)`、B 臂 `Ty(-700)`，兩臂相距 1400mm 面對面 — 依實際擺位調整。
4. **包覆球參數**（`gd_solver.cpp` 的 `BUBBLES_*` / `PEDESTAL_*` 常數）：RA610/RA605 的球座標與半徑，依實際機型確認。

---

## 授權

MIT
