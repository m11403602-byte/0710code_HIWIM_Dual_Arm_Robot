// =====================================================================
// cg_solver.hpp — 第 1 層: 純 Lagrangian 內層優化器 (共軛梯度)
//   = MATLAB Dual_Arm_Lagrangian_Con_v2
// =====================================================================
//   純 Lagrangian (S²-slack) + 共軛梯度 (Fletcher-Reeves CG) + 1D Newton 線搜索
//   ⚠ 與 ALM 譜系「不同的數學物件」:
//     決策變數 V = [X; λ; S] ∈ ℝ^1116  (λ/S 是決策變數, 非外層更新的乘子)
//     Lagrangian  L(V) = f(X) + w_d · λᵀ · (D(X) − θ·1 + S²)
//        (slack 化:  h_i = D_i − θ + s_i² = 0,  Bertsekas §3.3.2)
//     梯度  G = [G_X; G_λ; G_S]  為完整 KKT 一階殘差:
//        G_X = ∇f + w_d Σ λ_i ∇D_i        (stationarity, num_X)
//        G_λ = w_d · (D − θ + S²)          (primal feasibility, num_C)
//        G_S = 2 w_d · S ⊙ λ              (complementarity, num_C)
//     方向  d = −G + β·d_prev (FR; β=‖G‖²/‖pre_G‖²; 無 Hessian, 無 LDLT)
//     步長  α 由 1D Newton 線搜索 (LS_DELTA=0.01, 與 ALM 之 0.001 不同)
//
//   ⚠ 收斂 = phys_ok && stable_ok  (stat_ok 刻意不啟用):
//     純 Lagrangian 對 λ 線性 → 鞍點, ‖G‖ 不會收斂到 0;
//     λ 朝可行性漂移而非 KKT stationarity. 故以可行性 + 穩定為準.
//
//   [MATLAB] 對應 class: Dual_Arm_Lagrangian_Gradient_v2
//   座標/單位: 全 degree (與 MATLAB 一致); 距離 mm
//   幾何 (FK/球/calc_df/mask) 與 ALM/Newton 譜系位元一致 (同一套機器人模型)
// =====================================================================
#ifndef DUAL_ARM_LAG_CG_PLANNER_GD_SOLVER_HPP
#define DUAL_ARM_LAG_CG_PLANNER_GD_SOLVER_HPP

#include <Eigen/Dense>
#include <vector>
#include <string>

namespace dual_arm_lag_cg_planner
{

// [MATLAB] 包覆球定義: {link_id (0-indexed), 半徑, 局部座標 x/y/z}
struct BubbleDef {
  int    link_id;   // 掛在第幾連桿 (arm_frame, 0-indexed); 底盤球用 -1
  double radius;
  double cx, cy, cz;
};

// =====================================================================
// SolverLog — run_lag 回傳的完整追蹤
//   外層 avoidance_system 的 export 函式讀這些欄位
//   ⚠ 與 ALM 之 SolverLog 不同: 此處無 c/μ 排程, 改帶 KKT 殘差 + λ/S 診斷
// =====================================================================
struct SolverLog {
  // --- 標量結果 ---
  int    total_iter    = 0;     // 實際主迴圈迭代數
  int    converge_iter = -1;    // 收斂迭代 (-1 = 未收斂; phys_ok && stable_ok)
  int    diverge_iter  = -1;    // 發散迭代 (-1 = 未發散; ‖G‖>1e9 或 NaN/Inf)

  // --- 初末摘要 (供 export 對比) ---
  double max_D_init     = 0.0;
  double max_D_final    = 0.0;
  int    violation_init = 0;
  int    violation_final= 0;

  // --- 每迭代歷史 (長度 = total_iter) ---
  // (使用者要求往外傳的成本拆解)
  std::vector<double> L_history;       // Lagrangian L
  std::vector<double> f_history;       // 純平滑成本 f = pw·fA + (1−pw)·fB
  std::vector<double> fa_history;      // fA  (A 臂平滑成本)
  std::vector<double> fb_history;      // fB  (B 臂平滑成本)
  std::vector<double> penalty_history; // w_d · λᵀ · h
  std::vector<double> maxD_history;    // 每迭代 max(D)

  // 步進診斷
  std::vector<double> G_norm_history;  // ‖G‖ (全 V 梯度範數)
  std::vector<double> d_norm_history;  // ‖d‖
  std::vector<double> alpha_history;   // 線搜索步長 α

  // KKT 殘差分量 (純 Lagrangian 特有)
  std::vector<double> r_stat_history;  // ‖G_X‖           stationarity
  std::vector<double> r_prim_history;  // ‖h‖ = ‖D−θ+S²‖  primal feasibility
  std::vector<double> r_comp_history;  // ‖λ ⊙ S‖         complementarity
  std::vector<double> r_dual_history;  // ‖min(λ,0)‖       dual feasibility (λ≥0)
  std::vector<double> lam_neg_history; // Σ min(λ,0)       λ 漂負累計
  std::vector<double> kkt_history;     // √(r_stat²+r_prim²+r_comp²+r_dual²)
  std::vector<double> G_lam_norm_history; // ‖G_λ‖ (= r_prim·w_d, 分開記)
  std::vector<double> G_S_norm_history;   // ‖G_S‖

  // λ / S 範圍 + 計數
  std::vector<double> lam_max_history;
  std::vector<double> lam_min_history;
  std::vector<double> S_max_history;
  std::vector<double> S_min_history;
  std::vector<int>    active_count_history;    // sum(λ > 1e-6)
  std::vector<int>    violation_count_history; // sum(D > θ)
  std::vector<int>    ls_inner_history;        // 線搜索內層步數
  std::vector<int>    ls_fallback_history;     // 線搜索是否退 fallback (0/1)

  // --- 最終向量 ---
  Eigen::VectorXd final_D;    // (num_C) 最終全約束 D
  Eigen::VectorXd lam_final;  // (num_C)
  Eigen::VectorXd S_final;    // (num_C)
};

class CgSolver
{
public:
  // [MATLAB] Dual_Arm_Lagrangian_Gradient_v2(X, robotA_base, robotB_base, DANGER_THRESHOLD, path_weight)
  //   X: (P+2) x 12 矩陣 (頭 + P 中間點 + 尾), 每列 [A1..6, B1..6] (degree)
  CgSolver(const Eigen::MatrixXd& X,
           const Eigen::Matrix4d& robotA_base,
           const Eigen::Matrix4d& robotB_base,
           double danger_threshold,
           double path_weight,
           // 平滑權重 (MATLAB 預設全 1)
           double smooth_w          = 0.3,
           double smooth_w_H        = 1.0,
           double smooth_w_T        = 1.0,
           double smooth_w_neighbor = 1.0);

  // [MATLAB] run_newton (C++ 更名 run_lag): 回傳 log; X_final 由 get_X_final() 取得
  SolverLog run_lag();

  // ===== Getter =====
  // 取最終 X (num_X = 36 維), 即 V_final 的 X 段 (供 avoidance_system reshape)
  const Eigen::VectorXd& get_X_final() const { return X_final_; }
  // 取完整 V_final (num_X + 2·num_C = 1116 維)
  const Eigen::VectorXd& get_V_final() const { return V_final_; }
  int get_M() const { return M_; }          // 中間優化點數
  int get_num_D() const { return num_D_; }  // 每點約束數 (= K_AB)


  // [NEW] 純 Lagrangian 參數注入 (yaml → manager → avoidance_system → 此處; run_lag 前呼叫)
  //   ⚠ λ_0 與 ALM 之 μ_0 不同物理意義: 此處 λ 是決策變數初值
  void set_lag_params(double wd, double lam0, double s0,
                      double tol_phys_margin, double tol_stable,
                      int max_iter)
  {
    wd_              = wd;
    lam0_            = lam0;   s0_ = s0;
    TOL_PHYS_MARGIN_ = tol_phys_margin;
    TOL_STABLE_      = tol_stable;
    max_solver_iter_ = max_iter;
    rebuild_initial_V_();   // 以新 λ0/S0 重建 V_0 (覆蓋建構子初始化)
  }

  // [NEW] 迭代上限注入 (與 ALM 家族 set_iter_limits 介面對齊)
  void set_iter_limits(int max_iter)
  {
    if (max_iter > 0) max_solver_iter_ = max_iter;
  }

  // ===== Static 共用工具 (= MATLAB static 方法; 與 ALM/Newton 譜系位元一致) =====

  // [MATLAB] transmatrix(1, dir, deg): 旋轉矩陣 (角度 deg)
  static Eigen::Matrix4d make_rotation(char axis, double angle_deg);
  // [MATLAB] transmatrix(2, dir, val): 平移矩陣 (mm)
  static Eigen::Matrix4d make_translation(char axis, double dist_mm);

  // [MATLAB] calc_df(R1,R2,P1,P2): 球對危險因子 sj = exp(ln0.5/(Ri+Rj)^2 · d^2)
  static Eigen::MatrixXd calc_df(const Eigen::VectorXd& R1, const Eigen::VectorXd& R2,
                                 const Eigen::MatrixXd& P1, const Eigen::MatrixXd& P2);

  // [MATLAB] get_collision_masks(): 16x18 跨臂 mask (連桿級 cAB 展開到球級, K_AB=180)
  static Eigen::Array<bool, 16, 18> get_collision_masks();

  // [MATLAB] robot_arm_bubble_RA610_1476: RA610 FK → 16 球 (4 底盤 + 12 手臂)
  static void robot_arm_bubble_RA610(const Eigen::Matrix4d& T_base, const double J[6],
                                     Eigen::MatrixXd& bubble, Eigen::VectorXd& r,
                                     Eigen::Matrix4d& T_ee);
  // [MATLAB] robot_arm_bubble_RA605_710: RA605 FK → 18 球 (8 底盤 + 10 手臂)
  static void robot_arm_bubble_RA605(const Eigen::Matrix4d& T_base, const double J[6],
                                     Eigen::MatrixXd& bubble, Eigen::VectorXd& r,
                                     Eigen::Matrix4d& T_ee);

private:
  // ===== 維度 (建構期決定) =====
  int M_         = 0;   // 中間優化點數 (= P)
  int N_         = 6;   // 每臂軸數
  int num_D_     = 0;   // 每點約束數 (= K_AB, mask 過濾後)
  int num_X_     = 0;   // = 2 · M · N
  int num_C_     = 0;   // = M · num_D
  int total_dim_ = 0;   // = num_X + 2·num_C  (= 1116)

  // ===== 機器人/問題設定 =====
  Eigen::Matrix4d A_base_ = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d B_base_ = Eigen::Matrix4d::Identity();
  Eigen::Matrix<double, 2, 6> X_H_;   // 頭點 [A; B]
  Eigen::Matrix<double, 2, 6> X_T_;   // 尾點 [A; B]
  Eigen::MatrixXd oriPos_;            // (M x 12) 中間點原始關節角 (平滑參考)
  Eigen::VectorXd Xm_initial_;        // V_0 = [X_0; λ_0; S_0]   (total_dim)
  Eigen::VectorXd V_final_;           // (total_dim) 結果 V
  Eigen::VectorXd X_final_;           // (num_X) 結果 V 的 X 段

  double danger_threshold_ = 0.35;
  double path_weight_      = 0.5;
  double delta_            = 0.01;   // 有限差分擾動 h (FK 與 cost FD 共用, = MATLAB delta)

  // 平滑成本權重 (= MATLAB smooth_weights, 預設全 1)
  double smooth_w_          = 0.3;
  double smooth_w_H_        = 1.0;
  double smooth_w_T_        = 1.0;
  double smooth_w_neighbor_ = 1.0;

  // ===== 純 Lagrangian 參數 (= Gradient_v2 預設) =====
  double wd_               = 1.0;    // 對偶強度 (penalty 係數)
  double lam0_             = 30.0;   // λ_0  ⚠ 註解寫 10, 實際 code = 30
  double s0_               = 1.0;    // S_0  (S²=1)
  double TOL_PHYS_MARGIN_  = 0.01;   // max_D ≤ θ + margin
  double TOL_STABLE_       = 0.01;   // |Δmax_D| ≤ TOL_STABLE (原 MATLAB 值 0.005, 調整為 0.01)
  int    max_solver_iter_  = 500;


  // mask → 線性索引 (16x18 中為 true 的位置, column-major)
  std::vector<int> lin_idx_AB_;
  int K_AB_ = 0;

  // ===== 索引輔助 (V = [X; λ; S], 全 0-indexed) =====
  inline int idx_Xam(int m) const { return m * 2 * N_; }            // 第 m 點 A 臂 6 軸起點
  inline int idx_Xbm(int m) const { return m * 2 * N_ + N_; }       // 第 m 點 B 臂 6 軸起點
  inline int idx_Xm (int m) const { return m * 2 * N_; }            // 第 m 點 [Xa;Xb] 12 軸起點
  inline int idx_lam(int m) const { return num_X_ + m * num_D_; }   // 第 m 點 λ 起點 (V 全域)
  inline int idx_Sm (int m) const { return num_X_ + num_C_ + m * num_D_; } // 第 m 點 S 起點 (V 全域)
  inline int idx_lam_local(int m) const { return m * num_D_; }      // 第 m 點 λ 起點 (λ 區內)

  // ===== 內層數值方法 (= MATLAB instance 方法) =====
  Eigen::VectorXd compute_Dm(const Eigen::VectorXd& X, int m) const;     // 第 m 點 D (num_D)
  Eigen::VectorXd compute_Dx_all(const Eigen::VectorXd& X) const;        // 全點 D (num_C)
  void compute_D_cache(const Eigen::VectorXd& V,
                       Eigen::MatrixXd& D_base,                          // (num_D x M)
                       std::vector<Eigen::MatrixXd>& D_plus) const;      // M 個 (num_D x 12)
  Eigen::VectorXd compute_G_smooth(const Eigen::VectorXd& V) const;      // ∇_X f (num_X)
  Eigen::VectorXd compute_G(const Eigen::VectorXd& V,
                            const Eigen::MatrixXd& D_base,
                            const std::vector<Eigen::MatrixXd>& D_plus) const;  // 完整 KKT 殘差 (total_dim)
  double cost_function_L(const Eigen::VectorXd& V, Eigen::VectorXd& Dx_all_out) const;
  double cost_function_F(const Eigen::VectorXd& V) const;
  void   cost_function_F_split(const Eigen::VectorXd& V,
                               double& f, double& fa, double& fb) const;  // [NEW] 拆 fa/fb 往外傳
  double cost_Xm(const Eigen::MatrixXd& Xa, const Eigen::MatrixXd& Xori,
                 const Eigen::RowVectorXd& XH, const Eigen::RowVectorXd& XT) const;
  // [MATLAB] line_search_newton_1d: 回傳 α, 並透過 ref 帶出內層步數 / fallback 旗標
  double line_search_newton_1d(const Eigen::VectorXd& V, const Eigen::VectorXd& d,
                               int& ls_inner, bool& ls_fallback) const;

  // 以當前 lam0_/s0_/X 重建 V_0 (建構子 + set_lag_params 共用)
  void rebuild_initial_V_();

  // ===== 包覆球常數 (compile-time, 於 .cpp 用 {...} 初始化; 與 ALM 位元一致) =====
  static const std::vector<BubbleDef> PEDESTAL_A;  // RA610 底盤 4 球
  static const std::vector<BubbleDef> BUBBLES_A;   // RA610 手臂 12 球
  static const std::vector<BubbleDef> PEDESTAL_B;  // RA605 底盤 8 球
  static const std::vector<BubbleDef> BUBBLES_B;   // RA605 手臂 10 球
};

}  // namespace dual_arm_lag_cg_planner

#endif  // DUAL_ARM_LAG_CG_PLANNER_GD_SOLVER_HPP
