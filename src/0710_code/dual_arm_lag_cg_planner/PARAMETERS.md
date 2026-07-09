# 參數對照表 — dual_arm_lag_cg_planner

本規劃器所有可調參數皆在 `config/dual_arm_lag_cg_planning.yaml`（本表 = yaml 註解 + 補充說明）。
修改流程：編輯 yaml → 複製到 `hiwin_dual_arm/config/` → 重啟 move_group 生效（**不用重編**）。

---

## 常調參數

| 參數 | 預設 | 說明 |
|------|------|------|
| `path_weight` | `0.5` | A/B 臂成本權重 `pw·fA + (1-pw)·fB`，範圍 [0,1]。**越大**→RA610-1476-GC2(big_arm) 的避讓幅度越小、RA605-710-GC2(small_arm) 的避讓幅度越大；**越小**→big_arm 的避讓幅度越大、small_arm 的避讓幅度越小 |
| `danger_threshold` | `0.35` | 危險因子閾值（優化目標）；此值 + `collision_tolerance` = 碰撞邊界(0.5)，兩者之和不得超過 0.5（超過會漏判真碰撞）|
| `collision_tolerance` | `0.15` | 碰撞判定緩衝帶（= 碰撞邊界(0.5) − `danger_threshold`）|
| `fix_tolerance` | `0.1` | find_targets 的前後餘隙（fix_gap）比例 |
| `max_refinement_iter` | `15` | 外層修復最多輪數 |

---

## 軌跡平滑權重（平滑權重越大路徑越平滑）

| 參數 | 預設 | 說明 |
|------|------|------|
| `smooth_w` | `0.3` | 平滑項主權重：與原始組態距離 ‖Xm−Xm^ori‖² 的權重 |
| `smooth_w_H` | `1.0` | Head 端權重：第一關節組態與 Head 端距離 ‖X1−XH‖²（建議與 `smooth_w_T`、`smooth_w_neighbor` 一樣）|
| `smooth_w_T` | `1.0` | Tail 端權重：第三關節組態與 Tail 端距離 ‖X3−XT‖²（建議與 `smooth_w_H`、`smooth_w_neighbor` 一樣）|
| `smooth_w_neighbor` | `1.0` | 鄰點權重：相鄰兩組態距離 ‖X_(m+1)−X_m‖²（建議與 `smooth_w_H`、`smooth_w_T` 一樣）|

---

## 純 Lagrangian 參數（對應 Dual_Arm_Lagrangian_Con_v2）

> **調整指引**：
> - **來不及收斂**（跑滿 `lag_max_iter` 仍未達標）→ 提高 `lag_max_iter`。

| 參數 | 預設 | 說明 |
|------|------|------|
| `lag_lam0` | `30.0` | λ₀ 初值（λ 向量以此重建；原始值 30）|
| `lag_max_iter` | `500` | 主迴圈最大迭代（單層結構，無外/內層之分）。**調大**→給更多收斂預算 |

---

## 關節名前綴（依 SRDF 調整）

| 參數 | 預設 | 說明 |
|------|------|------|
| `joint_prefix_A` | `"big_joint_"` | A 臂 (RA610)：big_joint_1 ~ big_joint_6 |
| `joint_prefix_B` | `"small_joint_"` | B 臂 (RA605)：small_joint_1 ~ small_joint_6 |

---

## 時間參數化（在插件內處理）

| 參數 | 預設 | 說明 |
|------|------|------|
| `time_optimal` | `true` | true = TOTG 時間最佳化；false = 自訂等間隔 |
| `path_total_time` | `20.0` | （time_optimal=false 時）目標軌跡總時間（秒）|
| `min_time_interval` | `0.05` | （time_optimal=false 時）每點最小時間間隔（秒）|

- `time_optimal: true` → 用 TOTG 依關節速度/加速度限制計算時間戳（需在 joint_limits.yaml 設加速度限制，否則用預設值並印警告）。
- `time_optimal: false` → 等間隔 `dt = path_total_time / (點數−1)`，但 dt 不小於 `min_time_interval`（點數過多時實際總時間會超過 `path_total_time`）。

---

## 診斷輸出（預設全關）

| 參數 | 預設 | 說明 |
|------|------|------|
| `export_csv_prefix` | `"./lag_data"` | 匯出根目錄：`<prefix>/<unix秒>_CG/`（每次匯出自成時間戳資料夾，不互相覆蓋；export_level=0 時不匯出）|
| `export_level` | `0` | 0 = 完全不匯出（總開關）|
| | | 1 = 標配 6 檔（meta / summary / inner / danger_final / danger_rounds / targets）|
| | | 2 = 完整 9 檔（+ constraints_all / path_original / path_evolution）|
