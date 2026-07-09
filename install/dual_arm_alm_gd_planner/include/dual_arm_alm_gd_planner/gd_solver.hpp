// =====================================================================
// gd_solver.hpp — 第 1 層: 內層 GD 優化器 (= MATLAB Dual_Arm_Inequality_ALM_GD_v6)
// =====================================================================
//   ALM (PHR) 外層 + GD (Fletcher-Reeves 共軛梯度) 內層 + 1D Newton 線搜索
//   ⚠ GD 版「無 Hessian、無 LDLT、無方向記憶」: 方向 d = -G, 步長靠線搜索
//   數學模型與 CG_v7 / Newton_v6 完全相同 (僅內層求解方法為 GD 最陡下降)
//
//   [MATLAB] 對應 class: Dual_Arm_Inequality_ALM_GD_v6
//   座標/單位: 全 degree (與 MATLAB 一致); 距離 mm
//   FK 4x4 用 Eigen::Matrix4d (固定尺寸); 涉 num_D 的矩陣用動態尺寸
// =====================================================================
#ifndef DUAL_ARM_ALM_GD_PLANNER_GD_SOLVER_HPP
#define DUAL_ARM_ALM_GD_PLANNER_GD_SOLVER_HPP

#include <Eigen/Dense>
#include <vector>
#include <string>

namespace dual_arm_alm_gd_planner
{

// [MATLAB] 包覆球定義: {link_id (0-indexed), 半徑, 局部座標 x/y/z}
struct BubbleDef {
  int    link_id;   // 掛在第幾連桿 (arm_frame, 0-indexed)
  double radius;
  double cx, cy, cz;
};

// [MATLAB] run_alm 回傳的 log 結構 (對應 33 欄 log; verbose 區可選填)
//   avoidance_system 的 export 函式會讀這些欄位
struct SolverLog {
  // --- 標量結果 ---
  int    outer_iter       = 0;     // 外層 ALM 圈數
  int    total_inner      = 0;     // 內層總步數
  int    outer_break_iter = -1;    // 收斂圈 (-1 = 未收斂)
  int    diverge_iter     = -1;    // 發散圈 (-1 = 未發散)
  double c_final          = 5.0;   // 最終罰參數 c
  double final_G_norm     = 0.0;   // 最終 ||G||
  bool   feasibility_mode = false; // c 達上限 fallback

  // --- 外層歷史 (長度 = outer_iter) ---
  std::vector<double> L_history;         // Lagrangian
  std::vector<double> f_history_f;       // 平滑成本 f
  std::vector<double> f_history_fa;      // fA
  std::vector<double> f_history_fb;      // fB
  std::vector<double> penality_history;  // 罰項
  std::vector<double> maxD_history;      // 每圈 max_D
  std::vector<double> c_history;         // 每圈 c
  std::vector<double> mu_max_history;    // 每圈 max(mu)
  std::vector<double> vk_history;        // 對偶梯度 V_k
  std::vector<double> v_pure_history;    // 主可行性 v_pure
  std::vector<double> compl_history;     // 互補性
  std::vector<double> G_last_history;    // 每圈末步 ||G||
  std::vector<double> d_last_history;    // 每圈末步 ||d||
  std::vector<int>    inner_iter;        // 每圈內層步數

  // --- 內層歷史 (長度 = total_inner) ---
  std::vector<double> G_norm_history;
  std::vector<double> d_norm_history;
  std::vector<double> alpha_history;     // 線搜索步長

  // --- 最終向量 ---
  Eigen::VectorXd mu_final;   // (num_C)
  Eigen::VectorXd final_D;    // (num_C) 最後一輪全約束 D
};

class GdSolver
{
public:
  // [MATLAB] 建構函數 Dual_Arm_Inequality_ALM_GD_v6(X, robotA_base, robot_base, DANGER_THRESHOLD, path_weight)
  //   X: (P+2) x 12 矩陣 (頭 + P 中間點 + 尾), 每列 [A1..6, B1..6] (degree)
  GdSolver(const Eigen::MatrixXd& X,
           const Eigen::Matrix4d& robotA_base,
           const Eigen::Matrix4d& robotB_base,
           double danger_threshold,
           double path_weight,
           // 平滑權重 (帶預設值)
           double smooth_w          = 0.3,
           double smooth_w_H        = 1.0,
           double smooth_w_T        = 1.0,
           double smooth_w_neighbor = 1.0);

  // [MATLAB] run_alm: 回傳 log, X_final 由 get_X_final() 取得
  SolverLog run_alm();

  // Getter (外層只透過 getter 取結果)
  const Eigen::VectorXd& get_X_final() const { return X_final_; }
  int get_M() const { return M_; }          // 中間優化點數
  int get_num_D() const { return num_D_; }  // 每點約束數 (= K_AB)

  // [NEW] 迭代上限注入 (yaml 可調; K_inner 同步套用於首輪)
  void set_iter_limits(int k_outer, int k_inner)
  {
    if (k_outer > 0) K_outer_ = k_outer;
    if (k_inner > 0) { K_inner_ = k_inner; K_inner_first_ = k_inner; }
  }

  // [NEW] ALM 參數注入 (yaml → manager → avoidance_system → 此處; 須在 run_alm 前呼叫)
  //   mu_ 向量會以 mu0 重建 (覆蓋建構子的預設初始化)
  //   乘子更新為古典 PHR (Bertsekas 1982, eq.17), 無上界安全化 (reset 已移除)
  //   ⚠ c0 勿超過 c_max — 否則罰參數排程行為未定義
  void set_alm_params(double mu0, double c0, double c_max,
                      double beta_c, double gamma_v)
  {
    mu_ = Eigen::VectorXd::Constant(num_C_, mu0);
    c_ = c0; c_max_ = c_max;
    beta_c_ = beta_c; gamma_v_ = gamma_v;
  }

  // ===== Static 共用工具 (= MATLAB static 方法) =====

  // [MATLAB] transmatrix(1, dir, deg): 旋轉矩陣 (角度 deg)
  static Eigen::Matrix4d make_rotation(char axis, double angle_deg);
  // [MATLAB] transmatrix(2, dir, val): 平移矩陣 (mm)
  static Eigen::Matrix4d make_translation(char axis, double dist_mm);

  // [MATLAB] calc_df(R1,R2,P1,P2): 球對危險因子 sj = exp(ln0.5/(Ri+Rj)^2 * d^2)
  //   P1:(n1x3), P2:(n2x3), R1:(n1), R2:(n2) -> sj:(n1 x n2)
  static Eigen::MatrixXd calc_df(const Eigen::VectorXd& R1, const Eigen::VectorXd& R2,
                                 const Eigen::MatrixXd& P1, const Eigen::MatrixXd& P2);

  // [MATLAB] get_collision_masks(): 16x18 跨臂 mask (連桿級 cAB 展開到球級)
  static Eigen::Array<bool, 16, 18> get_collision_masks();

  // [MATLAB] robot_arm_bubble_RA610_1476: RA610 FK -> 16 球 (4 底盤 + 12 手臂)
  //   回傳 bubble:(16x3) 球心, r:(16) 半徑, T_ee:4x4 末端變換
  static void robot_arm_bubble_RA610(const Eigen::Matrix4d& T_base, const double J[6],
                                     Eigen::MatrixXd& bubble, Eigen::VectorXd& r,
                                     Eigen::Matrix4d& T_ee);
  // [MATLAB] robot_arm_bubble_RA605_710: RA605 FK -> 18 球 (8 底盤 + 10 手臂)
  static void robot_arm_bubble_RA605(const Eigen::Matrix4d& T_base, const double J[6],
                                     Eigen::MatrixXd& bubble, Eigen::VectorXd& r,
                                     Eigen::Matrix4d& T_ee);

private:
  // ===== 維度 (建構期決定) =====
  int M_     = 0;   // 中間優化點數 (= P)
  int num_D_ = 0;   // 每點約束數 (= K_AB, mask 過濾後)
  int num_X_ = 0;   // = 2 * M * 6
  int num_C_ = 0;   // = M * num_D

  // ===== 機器人/問題設定 =====
  Eigen::Matrix4d A_base_ = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d B_base_ = Eigen::Matrix4d::Identity();
  Eigen::Matrix<double, 2, 6> X_H_;   // 頭點 [A; B]
  Eigen::Matrix<double, 2, 6> X_T_;   // 尾點 [A; B]
  Eigen::MatrixXd oriPos_;            // (M x 12) 中間點原始關節角 (平滑參考)
  Eigen::VectorXd Xm_initial_;        // (num_X) 初始決策向量
  Eigen::VectorXd X_final_;           // (num_X) 結果

  double danger_threshold_ = 0.35;
  double path_weight_      = 0.5;
  double delta_            = 0.01;   // 差分步長 h

  // 平滑成本權重
  double smooth_w_          = 0.3;
  double smooth_w_H_        = 1.0;
  double smooth_w_T_        = 1.0;
  double smooth_w_neighbor_ = 1.0;

  // mask -> 線性索引 (16x18 中為 true 的位置, column-major)
  std::vector<int> lin_idx_AB_;  // 長度 = K_AB
  int K_AB_ = 0;

  // ===== ALM 參數 (= GD_v6 預設, 與 CG_v7 相同) =====
  Eigen::VectorXd mu_;          // (num_C) 乘子, 初始全 10
  double c_                   = 5.0;
  double c_max_               = 2000.0; // 三 solver 統一 (=yaml)
  double beta_c_              = 2.0;    // 三 solver 統一 (=yaml)
  double gamma_v_             = 0.5;
  double epsilon_v_           = 0.01;
  double epsilon_g_           = 0.1;
  double epsilon_compl_       = 0.1;
  double epsilon_inner_       = 1.0;
  double epsilon_inner_min_   = 0.1;
  double epsilon_inner_decay_ = 0.5;
  int    K_outer_             = 50;
  int    K_inner_             = 200;
  int    K_inner_first_       = 200;


  // ===== 內層數值方法 (= MATLAB instance 方法) =====
  Eigen::VectorXd compute_Dm(const Eigen::VectorXd& X, int m) const;          // 第 m 點 D (num_D)
  Eigen::VectorXd compute_Dx_all(const Eigen::VectorXd& X) const;             // 全點 D (num_C)
  Eigen::VectorXd compute_G_smooth(const Eigen::VectorXd& X) const;           // 平滑梯度 (num_X)
  Eigen::VectorXd compute_G_c(const Eigen::VectorXd& X,
                              const Eigen::MatrixXd& D_base, const std::vector<Eigen::MatrixXd>& D_plus,
                              const Eigen::VectorXd& mu_loc, double c_loc) const;   // 完整梯度
  void compute_D_cache(const Eigen::VectorXd& X,
                       Eigen::MatrixXd& D_base, std::vector<Eigen::MatrixXd>& D_plus) const;  // 2 回傳
  double cost_function_F(const Eigen::VectorXd& X) const;
  void   cost_function_F_split(const Eigen::VectorXd& X, double& f, double& fa, double& fb) const;
  double cost_Xm(const Eigen::MatrixXd& Xa, const Eigen::MatrixXd& Xori,
                 const Eigen::RowVectorXd& XH, const Eigen::RowVectorXd& XT) const;
  double cost_L_loc(const Eigen::VectorXd& X, const Eigen::VectorXd& mu_loc, double c_loc) const;
  double line_search_newton_1d(const Eigen::VectorXd& X, const Eigen::VectorXd& d,
                               const Eigen::VectorXd& mu_loc, double c_loc) const;

  // 索引輔助: 第 m 點 (0-indexed) 在 X 向量中 12 維的位置 (回傳 0-indexed 起點)
  inline int base_x(int m) const { return m * 12; }

  // ===== 包覆球常數 (compile-time, 於 .cpp 用 {...} 初始化) =====
  static const std::vector<BubbleDef> PEDESTAL_A;  // RA610 底盤 4 球
  static const std::vector<BubbleDef> BUBBLES_A;   // RA610 手臂 12 球
  static const std::vector<BubbleDef> PEDESTAL_B;  // RA605 底盤 8 球
  static const std::vector<BubbleDef> BUBBLES_B;   // RA605 手臂 10 球
};

}  // namespace dual_arm_alm_gd_planner

#endif  // DUAL_ARM_ALM_GD_PLANNER_GD_SOLVER_HPP
