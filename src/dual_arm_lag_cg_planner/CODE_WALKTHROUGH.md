# CODE_WALKTHROUGH.md — 程式碼閱讀順序 + 完整呼叫圖解

這份文件是給「第一次要讀這個套件原始碼的人」看的**閱讀指南**，重點是回答：
「這一堆 `.cpp` 檔案裡的 function 到底誰先跑、誰呼叫誰、建構子做了什麼」。

演算法的數學公式、參數表已經寫在 [`README.md`](README.md)（架構總覽/流程圖）
和 [`PARAMETERS.md`](PARAMETERS.md)（每個可調參數的意義），本文件**不重複**那些
內容，而是聚焦在「程式碼執行順序」這個切入點，並且已經把對應的
**呼叫順序註解直接寫進 4 個 `.cpp` 檔案本身**（搜尋 `【呼叫順序 X】` 就能在
編輯器裡定位到每個 function）。建議搭配編輯器開著本文件 + 4 個 `.cpp` 對照讀。

---

## 0. 先建立一個大架構：4 個檔案分別負責什麼

```
planner_manager.cpp   ← MoveIt2 的「大門」，唯一被 MoveIt 呼叫的檔案
        │
        ▼
avoidance_system.cpp  ← 第 2 層「外層」：碰撞修復迴圈（多輪，找危險段→修）
        │
        ▼
cg_solver.cpp          ← 第 1 層「內層」：純 Lagrangian + 共軛梯度優化器
        │
        ▼
data_io.cpp             ← 最底層：CSV 寫檔工具（純 I/O，不含演算法）
```

**由上而下依賴**：`planner_manager` 用 `avoidance_system`，`avoidance_system`
用 `cg_solver`，`avoidance_system` 也用 `data_io` 做匯出。`cg_solver` 和
`data_io` 互相都不知道對方存在。閱讀順序建議照這個箭頭方向讀下去。

---

## 1. 從「使用者按下 RViz 的 Plan 按鈕」開始：完整時間軸

以下是**一次規劃請求**從頭到尾，函式被呼叫的**真實先後順序**（不是檔案裡的
排列順序，是「執行時」的順序）：

```
【啟動時只跑一次】
DualArmLagCgPlannerManager::initialize()                    planner_manager.cpp
  └─ load_parameters()                                       (讀 yaml 一次)

【每次按 Plan 都重跑一次，由這裡開始】
① DualArmLagCgPlannerManager::getPlanningContext()           planner_manager.cpp
     ├─ load_parameters()                                     (重讀 yaml，讓改參數不用重啟)
     └─ new DualArmPlanningContext(...)                       (把參數複製進 context)

② DualArmPlanningContext::solve()                             planner_manager.cpp
     ├─ 取出 MoveIt 的 start_state / goal_state (RobotState)
     ├─ radian → degree，組出 A_wp (2x6)、B_wp (2x6)
     │
     ├─③ new AvoidanceSystem(A_wp, B_wp, ...)                 avoidance_system.cpp
     │      └─ generate_initial_trajectory()
     │           └─ clamped_cubic_spline()      ← 產生「起點→終點」的初始弧線
     │
     ├─ optimizer.set_lag_params(...)                          (灌入 yaml 的 λ/S 初值等)
     │
     ├─④ optimizer.run_optimization()                          avoidance_system.cpp
     │      ├─ check_collision(初始軌跡)
     │      │    → 安全 or 起終點本身太危險 → 直接 return
     │      └─ 否則進「外層修復迴圈」(最多 max_refinement_iter 輪):
     │           每一輪依序:
     │           (a) find_collision_targets()      找最危險段的 5 個特徵點
     │           (b) run_solver_global()
     │                 └─⑤ new CgSolver(5點中的12維數據, ...)   cg_solver.cpp
     │                       └─ rebuild_initial_V_()             (組 V0=[X0;λ0;S0])
     │                 └─⑥ solver.run_lag()   ★核心優化主迴圈★   cg_solver.cpp
     │                       (細節見下面第 2 節)
     │           (c) regenerate_trajectory_global()   把優化結果重新 spline 回整段軌跡
     │           (d) check_collision(新軌跡)          還有碰撞就回 (a)，沒有就跳出迴圈
     │
     ├─ (可選) optimizer.export_unified()                       avoidance_system.cpp
     │      └─ write_csv() / write_csv_labeled()                 data_io.cpp
     │
     ├─ degree → radian，把優化後的軌跡點填回 RobotTrajectory
     └─ TOTG 時間參數化 (或等間隔)，回傳給 MoveIt

③ DualArmPlanningContext::solve(DetailedResponse&)             planner_manager.cpp
     └─ 只是包一層，內部直接呼叫上面②的 solve()
```

**一句話總結**：`planner_manager` 把 MoveIt 的請求「翻譯」成兩個 2x6 矩陣，
交給 `AvoidanceSystem` 這個「不依賴 MoveIt、純數學」的物件去算，算完再把
結果「翻譯」回 MoveIt 看得懂的 `RobotTrajectory`。真正的避障運算全部發生在
`AvoidanceSystem::run_optimization()` 和它呼叫的 `CgSolver::run_lag()` 裡。

---

## 2. 內層 `CgSolver::run_lag()` 主迴圈，一步步在做什麼

這是整個套件裡數學最密集的地方，`cg_solver.cpp` 頂端已經寫了完整的
Step 1~12 呼叫順序表，這裡用白話再解釋一次每一步「在幹嘛」：

| Step | 呼叫的函式 | 這步在做什麼 | 為什麼要這樣做 |
|---|---|---|---|
| 1 | `compute_D_cache()` → `compute_Dm()` | 對每個中間點做一次 FK+碰撞距離計算，順便對每個關節做 +h 微擾算敏感度 | 之後算梯度不用重複呼叫 FK，省算力 |
| 2 | `compute_G()` → `compute_G_smooth()` → `cost_function_F()` → `cost_Xm()` | 組出完整 KKT 一階殘差 `G = [G_X; G_λ; G_S]` | 這是「往哪個方向走能讓 Lagrangian 變小」的核心資訊 |
| 3 | (迴圈內聯，非獨立函式) | 共軛梯度方向：第一輪 `d=-G`，之後 Fletcher-Reeves 公式 `d=-G+β·d_prev` | 比單純最陡下降 (`d=-G`) 收斂快，且不用像 Newton 法算 Hessian |
| 4 | `line_search_newton_1d()` → `cost_function_L()` (×多次) | 沿著方向 `d` 用 1D 牛頓法找最佳步長 `α` | 走一步该走多遠，不會走過頭也不會走太保守 |
| 5-6 | (迴圈內聯) | `V ← V + α·d`，重新評估 Lagrangian 值 `L` 與最大危險值 `max_D` | 更新決策變數 |
| 7 | (迴圈內聯) | 把這輪的 `G_norm`/`d` 存成下一輪 CG 公式要用的「上一輪」 | Fletcher-Reeves 需要記住上一輪的梯度範數 |
| 8-11 | (迴圈內聯) | 拆解 KKT 各分量殘差（stationarity/primal/complementarity/dual）、寫進 `SolverLog` | 供 CSV 診斷輸出、也用於下面的收斂判定 |
| 12 | (迴圈內聯) | 判斷 `phys_ok && stable_ok` 是否成立 → 收斂就跳出迴圈 | 見下方「為什麼收斂條件跟 ALM 不一樣」 |

### 為什麼是「純 Lagrangian」而不是 ALM？收斂條件為什麼不看 ‖G‖？

- ALM（增廣拉格朗日，其他 5 個規劃器用的方法）把「罰參數 c」和「乘子 μ」放在
  **外層**逐步更新，內層只解一個固定罰參數下的子問題。
- 這個 `dual_arm_lag_cg_planner` 走的是**純 Lagrangian + Slack 變數**：把
  乘子 `λ` 和鬆弛變數 `S`（滿足 `S²≥0` 讓不等式 `D≤θ` 變成等式
  `D-θ+S²=0`）**直接當成決策變數**，跟關節角 `X` 一起放進同一個向量
  `V=[X; λ; S]`，一次性用共軛梯度解到底，沒有外層更新迴圈。
- 這種寫法在數學上會走到一個 **鞍點 (saddle point)**，不是純粹的極小值 ——
  對 `λ` 方向是線性的，所以理論上 `‖G‖` 不會收斂到 0（一直有殘留梯度）。
  因此程式碼裡**刻意不用** `‖G‖<tol` 當收斂條件，改用「物理可行性
  `max_D ≤ θ+margin`」加「數值穩定 `|Δmax_D|<tol`」兩個條件一起成立才算
  收斂（`cg_solver.cpp` Step 12，對應 `cg_solver.hpp` 開頭那段註解）。

---

## 3. 外層 `AvoidanceSystem::run_optimization()`，為什麼要「修復迴圈」而不是一次解到底

一次把整條軌跡（可能幾十到幾百個點）全部丟進 `CgSolver` 優化，決策變數維度
會爆炸（`num_X_ = 2 × 點數 × 6`），優化又慢又容易數值不穩。所以外層採用
「**只修最危險的一小段**」策略：

1. 先用 clamped cubic spline 生一條**完全不管碰撞**的初始弧線（起點到終點，
   兩個 waypoint 內插）。
2. 逐步檢查每個時間步的危險值，若整段都低於閾值就直接收工（`run_optimization`
   最開頭的 fast-path）。
3. 若有碰撞，`find_collision_targets()` 只挑出「最危險的一段連續區間」，
   在這段裡取 5 個特徵點（頭、q1、危險峰值、q3、尾）。
4. 只把**中間 3 個點**（q1/peak/q3，共 3×12=36 維）丟進 `CgSolver` 優化，
   頭尾 2 個點固定不動（維持軌跡邊界連續）。
5. 優化完，`regenerate_trajectory_global()` 只針對這一小段重新 spline，
   跟前後沒改動的部分拼接回一條完整軌跡（`C1` 邊界斜率對齊避免銜接處抖動）。
6. 重新檢查整條軌跡有沒有碰撞：還有 → 回到步驟 3 修下一個危險段；
   沒有 → 完成；輪數超過 `max_refinement_iter` 仍有碰撞 → 回報失敗。

這就是為什麼函式名稱有「Head / q1 / peak / q3 / Tail」這種命名 —— 它們是
`find_collision_targets()` 在最危險區間裡取樣出的 5 個代表點，不是隨便取的。

---

## 4. 「建構子做了什麼」— 常見疑問集中回答

### `AvoidanceSystem` 的建構子
只做兩件事：(1) 設定兩臂底座的固定變換矩陣 `robotA_base_`/`robotB_base_`
（寫死 1400mm 對裝、面對面轉 180°）；(2) 呼叫
`generate_initial_trajectory()` 生一條初始弧線。**不做任何碰撞檢查或優化**
——那些要等外部呼叫 `run_optimization()` 才會發生。

### `CgSolver` 的建構子
先算出這次優化問題的「維度」（`M_`=中間點數、`num_D_`=每點約束數、
`num_X_`/`num_C_`/`total_dim_`），再記住頭尾邊界點和中間點原始角度，最後
呼叫 `rebuild_initial_V_()` 把決策變數初值 `V0=[X0; λ0; S0]` 組出來。
**每一輪外層修復都會 new 一個全新的 `CgSolver`**（不是重複使用同一個物件
反覆優化），因為每一輪要優化的點、維度都不同。

### `set_lag_params()` 為什麼會觸發 `rebuild_initial_V_()`？
因為 `λ0`/`S0` 這兩個初值是在建構子裡就用預設值算過一次 V0 了；
`set_lag_params()` 從 yaml 讀到「真正該用的值」後，必須重新組一次 V0
才能覆蓋掉建構子的預設值，否則優化會從錯的起點開始。

---

## 5. 想追某個具體問題，該看哪個函式？

| 我想知道... | 去看這個函式 |
|---|---|
| 兩顆球之間怎麼定義「危險」 | `cg_solver.cpp` 的 `calc_df()` |
| 手臂的正向運動學怎麼算 | `cg_solver.cpp` 的 `robot_arm_bubble_RA610`/`RA605` |
| 哪些球對需要檢查、哪些跳過 | `cg_solver.cpp` 的 `get_collision_masks()` |
| 收斂條件到底是什麼 | `cg_solver.cpp` 的 `run_lag()` Step 12 |
| 為什麼優化完軌跡會「接得上」原本沒改的部分 | `avoidance_system.cpp` 的 `regenerate_trajectory_global()`（C1 邊界斜率） |
| 危險段的 5 個特徵點怎麼挑出來的 | `avoidance_system.cpp` 的 `find_collision_targets()` |
| MoveIt 的 radian 何時轉成演算法用的 degree | `planner_manager.cpp` 的 `solve()` 第 4 步 |
| CSV 匯出的每個檔案對應什麼資料 | `avoidance_system.cpp` 的 `export_unified()`（本身也有逐段編號註解） |
| 關節名 / 底座間距 / 球模型如何換成別的機型 | 見工作區根目錄的 [`PORTING_NEW_ROBOT.md`](../../PORTING_NEW_ROBOT.md) |

---

## 6. 與其他 5 個規劃器套件的關係

`dual_arm_lag_cg_planner` 是「純 Lagrangian 譜系」的 CG（共軛梯度）版本。
同譜系還有 `dual_arm_lag_newton_planner`（用 Newton 法找方向而非 CG）、
`dual_arm_lag_gd_planner`（最陡下降）。另一個「ALM 譜系」
（`dual_arm_alm_*_planner`）用增廣拉格朗日 + 外層乘子更新，是不同的數學
模型，程式結構（4 個檔案、呼叫順序）幾乎一樣，但 `cg_solver`/`avoidance_system`
內部數學不同，**不能把兩譜系的 yaml 參數互相套用**（主手冊
[`README.md`](../../README.md) §2 已提過這點）。
