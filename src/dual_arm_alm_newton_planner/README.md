# dual_arm_alm_newton_planner

雙臂避障路徑規劃器 — ROS 2 Humble / MoveIt2 插件。

使用 C++17 + Eigen3 實作。內層採用 **ALM（增廣拉格朗日）+ Newton 法（完整 Hessian + LDLT）**。

---

## 演算法概觀

兩隻機械臂（RA610 16 球 / RA605 18 球包覆模型），給定起點與終點關節角，求一條兩臂互不碰撞的關節空間軌跡。

採雙層最佳化：

```
外層 (avoidance_system):  生成初始軌跡 -> 碰撞偵測 -> 找危險段 5 點 ->
                          呼叫內層優化 -> Spline 重建 -> 重新檢查 (最多 max_refinement_iter 輪，可調參數)
內層 (newton_solver):         ALM 外層 (罰參數/乘子更新) + Newton 內層 (LDLT)
```

危險因子閾值 `danger_threshold = 0.35`、碰撞判定餘隙 `collision_tolerance = 0.15`（皆 yaml 預設，須為正值）。碰撞邊界(0.5) = `danger_threshold + collision_tolerance`；**設定時兩者之和不得超過碰撞邊界(0.5)**（0.5 = 兩球相切，超過即會漏判真碰撞）。

---

## 架構：單一 .so

所有原始碼（核心演算法 + MoveIt 插件介面）編進**單一共享庫** `libdual_arm_alm_newton_planner.so`。

| 組成 | 角色 |
|------|------|
| `newton_solver` | 第 1 層: 內層 Newton 求解器 + FK + 包覆球 + 危險因子 |
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
│ 取 3 個內部點 (q1, peak, q3) 組成決策變數 X (36 維)                   │
│ 呼叫內層優化器 run_alm(X0) → X*                                      │
│        │                                                            │
│        ▼                                                            │
│ regenerate_trajectory_global(): 用 X* 做局部 Spline 重建整段軌跡      │
│        │                                                            │
│        ▼                                                            │
│ 重新計算 max_D(t)，回到「危險判定」 (最多 max_refinement_iter 輪)     │
└────────────────────────────────────────────────────────────────────┘
```

### 內層：ALM 牛頓求解器 (`NewtonSolver::run_alm`)

> 圖中「參數名=數值」為 yaml 預設值示例，實際以 config 為準。

```
┌─────────────────────────────────────────────────────────────────────┐
│ 初始化: mu = alm_mu0(=10), c = alm_c0(=5)                             │
│        │                                                             │
│        ▼                                                             │
│  ┌── 外層 ALM 迴圈 (最多 alm_k_outer=50 圈) ─────────────────────┐    │
│  │   ┌── 內層 Newton 迭代 (最多 alm_k_inner=200 步) ───────────┐ │    │
│  │   │  計算 G(X) = ∇f + Σ mu_i · max(0, g_i(X)) · ∇g_i       │ │    │
│  │   │  計算 H = H_smooth + H_collision (數值 Hessian)          │ │    │
│  │   │  解 d = -(H \ G)   (Eigen LDLT 分解)                    │ │    │
│  │   │  X ← X + d   (alpha = 1, 純 Newton 步)                  │ │    │
│  │   │  ‖G‖ < 內層容差 (逐圈收緊)?  ──否──▶ 繼續下一步           │ │    │
│  │   └── 是 ─────────────────────────────────────────────────┘ │    │
│  │        ▼                                                     │    │
│  │  更新乘子 mu_i ← max(0, mu_i + c·g_i(X))                      │    │
│  │  ‖V_k‖ > alm_gamma_v(=0.5)·‖V_k-1‖ ?                          │    │
│  │      ──是──▶ c ← alm_beta_c(=2.0)·c (上限 alm_c_max=2000)     │    │
│  │        ▼                                                     │    │
│  │  收斂判定: 可行性、梯度範數、互補性皆低於容差                     │    │
│  │      ──否──▶ 回到內層 Newton 迭代                               │    │
│  └── 是 ──▶ 回傳 X*、SolverLog ─────────────────────────────────┘    │
└─────────────────────────────────────────────────────────────────────┘
```

### 主要變數

| 變數 | 意義 | 維度 |
|------|------|------|
| `X` | 決策變數：危險段 3 個內部控制點 × 雙臂各 6 軸（degree） | 36 |
| `D_m` | 第 m 個控制點的危險因子向量 | `num_D_` |
| `g_i(X)` | 不等式約束：`D_i(X) − danger_threshold` | 共 `num_C_` 個 |
| `G` | Lagrangian 對 X 的梯度 | 36 |
| `d` | Newton 方向 `-(H\G)`（LDLT） | 36 |
| `mu` | ALM 乘子 | `num_C_` |
| `c` | ALM 罰參數 | scalar |

各參數的預設值與說明詳見 [PARAMETERS.md](PARAMETERS.md)。

---

## 目錄結構

- **根目錄檔案**（`CMakeLists.txt` / `package.xml` / `*.xml` / `README.md` / `PARAMETERS.md`）：**建置與描述層**——定義怎麼編譯和對外說明文件。
- **`config/`**：**執行期參數層**——存放 move_group 啟動時讀入的 yaml 參數，需複製到 `hiwin_dual_arm/config/`。
- **`include/dual_arm_alm_newton_planner/`**：**介面宣告層**。
- **`src/`**：**演算法實作層**。

```
dual_arm_alm_newton_planner/
├── CMakeLists.txt
├── package.xml
├── dual_arm_alm_newton_planner.xml
├── README.md
├── PARAMETERS.md
├── config/
│   └── dual_arm_alm_newton_planning.yaml
├── include/dual_arm_alm_newton_planner/
│   ├── newton_solver.hpp
│   ├── avoidance_system.hpp
│   ├── data_io.hpp
│   └── planner_manager.hpp
└── src/
    ├── newton_solver.cpp
    ├── avoidance_system.cpp
    ├── data_io.cpp
    └── planner_manager.cpp
```

| 檔案 | 功能 |
|------|------|
| `CMakeLists.txt` | 建置設定：編譯單一 .so |
| `package.xml` | ROS 2 套件描述與相依宣告 |
| `dual_arm_alm_newton_planner.xml` | pluginlib 插件描述（讓 MoveIt2 動態載入） |
| `README.md` | 本文件 |
| `PARAMETERS.md` | 參數對照表（可調參數） |
| `config/dual_arm_alm_newton_planning.yaml` | 規劃器參數（需複製到 `hiwin_dual_arm/config/`） |
| `newton_solver.hpp/.cpp` | 第 1 層：FK、包覆球、危險因子、ALM + Newton 內層求解 |
| `avoidance_system.hpp/.cpp` | 第 2 層：外層碰撞修復迴圈、Spline 重建、CSV 匯出 |
| `data_io.hpp/.cpp` | CSV 寫入工具 |
| `planner_manager.hpp/.cpp` | 第 3 層：MoveIt2 插件介面（radian↔degree、時間參數化） |

---

## 編譯

放進 ROS 2 workspace 的 `src/` 後：

```bash
cd ~/ros2_ws
colcon build --packages-select dual_arm_alm_newton_planner
source install/setup.bash
```

需求：ROS 2 Humble、MoveIt2、Eigen3、C++17 編譯器。

### ⚠️ 編譯選項：使用 `-O3`，**不要用 `-march=native`**

CMake 預設 Release `-O3 -DNDEBUG`（**刻意不含 `-march=native`**，會與 MoveIt 的 Eigen 記憶體對齊衝突而崩潰）。

---

## 使用方式（MoveIt2 插件）

> 前提：與 `hiwin_dual_arm`（機器人描述 + MoveIt 設定）同 workspace。

1. 把本包 `config/dual_arm_alm_newton_planning.yaml` 複製到 `hiwin_dual_arm/config/`（檔名依規劃器而不同）。
   檔內已含 `planning_plugin` 與 `request_adapters` 完整設定，複製即可、無需再改 pipeline。
2. 啟動（例）：

```bash
ros2 launch hiwin_dual_arm brain.launch.py
```

規劃請求需含 joint goal constraints。

**改參數**：編輯 yaml → 重新複製到 `hiwin_dual_arm/config/` → 重啟 move_group 生效（**不用重編**）。

---

## CSV 匯出

對外唯一入口：`export_unified(prefix, level)`。輸出到 `prefix/<timestamp>_Newton/` 目錄（每次匯出自成一個時間戳資料夾，不互相覆蓋）。

| level | 檔案 | 內容 |
|:---:|------|------|
| **0** | （無） | 完全不匯出（總開關；yaml 目前預設） |
| **1** | `meta.csv` | 參數快照（key/value，含 ALM 五參數） |
| | `summary.csv` | 每輪修復計分板（輪次/迭代/L/f/maxD/c_final…） |
| | `inner.csv` | 內層逐步串接（round / inner_step / ‖G‖ / ‖d‖ / alpha） |
| | `danger_final.csv` | 初始 vs 最終每步 MaxD 對照 |
| | `danger_rounds.csv` | 長表 round／step／MaxD_in／MaxD_out |
| | `targets.csv` | 每輪 5 特徵點（索引 + D 值 + 優化前後關節角） |
| **2** | 以上 6 檔 + `constraints_all.csv` | 全約束 D 值大表 |
| | `path_original.csv` | 初始軌跡全點 |
| | `path_evolution.csv` | 長表 round／step／A1..B6（每輪軌跡演化） |

MoveIt 路徑由 yaml 的 `export_csv_prefix`（預設 `./alm_data`）與 `export_level` 控制（預設 0 = 不匯出；要留資料改 1 或 2 後重啟 move_group）。所有 CSV 可用 Excel、Python `pandas.read_csv` 直接開啟。

---

## 需要調整的地方

部署前請依實際機器人調整：

1. **關節名**（`src/planner_manager.cpp` 的 `solve()`）：目前用 A 臂 `big_joint_1~6`、B 臂 `small_joint_1~6`，改成你 SRDF 的實際關節名。
2. **planning group**：solve 用的`group_name`，確認 SRDF 的 group 設定。
3. **機器人底座**（`avoidance_system.cpp` 建構函數）：目前 A 臂 `Ty(700)Rz(180)`、B 臂 `Ty(-700)`，兩臂相距 1400mm 面對面 — 依實際擺位調整。
4. **包覆球參數**（`newton_solver.cpp` 的 `BUBBLES_*` / `PEDESTAL_*` 常數）：RA610/RA605 的球座標與半徑，依實際機型確認。

---

## 授權

MIT
