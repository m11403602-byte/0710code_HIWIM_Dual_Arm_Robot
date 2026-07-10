// =====================================================================
// gd_solver.cpp — 第 1 層 GD 求解器實作 (= MATLAB Dual_Arm_Inequality_ALM_GD_v6)
// =====================================================================
#include "dual_arm_alm_gd_planner/gd_solver.hpp"

#include <cmath>
#include <iostream>
#include <algorithm>
#include <limits>

namespace dual_arm_alm_gd_planner
{

// =====================================================================
// 包覆球常數 (= MATLAB robot_arm_bubble 內的 arm_r/arm_p/arm_frame、ped_*)
// =====================================================================

// [MATLAB] RA610 底盤 4 球 (R=320, 中心固定在 T_base 座標系)
const std::vector<BubbleDef> GdSolver::PEDESTAL_A = {
  { -1, 320.0,  220.0,  -65.0, 55.0 },
  { -1, 320.0,  220.0,  296.0, 55.0 },
  { -1, 320.0, -220.0,  296.0, 55.0 },
  { -1, 320.0, -220.0,  -64.0, 55.0 },
};

// [MATLAB] RA610 手臂 12 球 {link_id (arm_frame, 0-indexed), R, [x,y,z]}
const std::vector<BubbleDef> GdSolver::BUBBLES_A = {
  { 0, 280.0,    0.0,   0.0, 145.0 },
  { 1, 300.0,  -15.0,   0.0, -70.0 },
  { 2, 115.0,  160.0,   0.0,  80.0 },
  { 2, 115.0,  160.0,   0.0, 210.0 },
  { 2, 115.0,  160.0,   0.0, 340.0 },
  { 2, 115.0,  160.0,   0.0, 460.0 },
  { 3, 250.0,   60.0,  60.0,  60.0 },
  { 4, 110.0,    0.0, 290.0,   0.0 },
  { 4, 110.0,    0.0, 430.0,   0.0 },
  { 4, 110.0,    0.0, 560.0,   0.0 },
  { 5, 105.0,    0.0,   0.0,   0.0 },
  { 6, 100.0,    0.0,   0.0,   0.0 },
};

// [MATLAB] RA605 底盤 8 球 (R=350)
const std::vector<BubbleDef> GdSolver::PEDESTAL_B = {
  { -1, 350.0,  170.0,  290.0, 661.0 },
  { -1, 350.0, -170.0,  290.0, 661.0 },
  { -1, 350.0, -170.0, -230.0, 661.0 },
  { -1, 350.0,  170.0, -230.0, 661.0 },
  { -1, 350.0,  185.0,  250.0, 231.0 },
  { -1, 350.0, -185.0,  250.0, 231.0 },
  { -1, 350.0, -185.0, -250.0, 231.0 },
  { -1, 350.0,  185.0, -250.0, 231.0 },
};

// [MATLAB] RA605 手臂 10 球
const std::vector<BubbleDef> GdSolver::BUBBLES_B = {
  { 0, 175.0,    0.0,    0.0,   85.0 },
  { 1, 135.0,    0.0,    0.0, -135.0 },
  { 2, 150.0,  -16.0,    0.0,   25.0 },
  { 2, 150.0,  -16.0,    0.0,  120.0 },
  { 2, 150.0,  -16.0,    0.0,  240.0 },
  { 3, 135.0,    0.0,   30.0,   10.0 },
  { 4,  95.0,    0.0,  130.0,    0.0 },
  { 4,  95.0,    0.0,  230.0,    0.0 },
  { 5,  95.0,    0.0,  -23.0,    0.0 },
  { 6,  75.0,    0.0,  -36.5,    0.0 },
};

// =====================================================================
// transmatrix -> make_rotation / make_translation
// =====================================================================

// [MATLAB] transmatrix(mode=1, dir, deg): 旋轉 (角度 deg)
Eigen::Matrix4d GdSolver::make_rotation(char axis, double angle_deg)
{
  const double cv = std::cos(angle_deg * M_PI / 180.0);
  const double sv = std::sin(angle_deg * M_PI / 180.0);
  Eigen::Matrix4d m = Eigen::Matrix4d::Identity();
  if (axis == 'x' || axis == 'X') {
    m(1,1)= cv; m(1,2)=-sv; m(2,1)= sv; m(2,2)= cv;
  } else if (axis == 'y' || axis == 'Y') {
    m(0,0)= cv; m(0,2)= sv; m(2,0)=-sv; m(2,2)= cv;
  } else if (axis == 'z' || axis == 'Z') {
    m(0,0)= cv; m(0,1)=-sv; m(1,0)= sv; m(1,1)= cv;
  }
  return m;
}

// [MATLAB] transmatrix(mode=2, dir, val): 平移 (mm)
Eigen::Matrix4d GdSolver::make_translation(char axis, double dist_mm)
{
  Eigen::Matrix4d m = Eigen::Matrix4d::Identity();
  if (axis == 'x' || axis == 'X')      m(0,3) = dist_mm;
  else if (axis == 'y' || axis == 'Y') m(1,3) = dist_mm;
  else if (axis == 'z' || axis == 'Z') m(2,3) = dist_mm;
  return m;
}

// =====================================================================
// calc_df: 球對危險因子
// =====================================================================
// [MATLAB] sj_ij = exp( ln(0.5)/(R_i+R_j)^2 * d_ij^2 ); 手動雙層迴圈取代 pdist2
Eigen::MatrixXd GdSolver::calc_df(const Eigen::VectorXd& R1, const Eigen::VectorXd& R2,
                                  const Eigen::MatrixXd& P1, const Eigen::MatrixXd& P2)
{
  const int n1 = static_cast<int>(P1.rows());
  const int n2 = static_cast<int>(P2.rows());
  const double log05 = std::log(0.5);
  Eigen::MatrixXd sj(n1, n2);
  for (int i = 0; i < n1; ++i) {
    for (int j = 0; j < n2; ++j) {
      const double dx = P1(i,0) - P2(j,0);
      const double dy = P1(i,1) - P2(j,1);
      const double dz = P1(i,2) - P2(j,2);
      const double d2  = dx*dx + dy*dy + dz*dz;
      const double Rij = R1(i) + R2(j);
      sj(i,j) = std::exp(log05 / (Rij * Rij) * d2);
    }
  }
  return sj;
}

// =====================================================================
// get_collision_masks: 16x18 跨臂 mask
// =====================================================================
// [MATLAB] sA/sB 球->連桿映射; cAB 連桿級 (行 4~8 vs 全部 = 1); mask = cAB(sA,sB)
Eigen::Array<bool, 16, 18> GdSolver::get_collision_masks()
{
  // 球 -> 連桿 ID (1-indexed, 同 MATLAB)
  static const int sA[16] = {1,1,1,1, 2, 3, 4,4,4,4, 5, 6,6,6, 7, 8};
  static const int sB[18] = {1,1,1,1,1,1,1,1, 2, 3, 4,4,4, 5, 6,6, 7, 8};
  // cAB 8x8: 行 1~3 (L0~L2 底盤側) 全 0, 行 4~8 全 1 (1-indexed)
  auto cAB = [](int rowLink, int /*colLink*/) -> bool {
    return rowLink >= 4;   // [MATLAB] cAB 第 4~8 列為 1, 其餘 0
  };
  Eigen::Array<bool, 16, 18> mask;
  for (int i = 0; i < 16; ++i)
    for (int j = 0; j < 18; ++j)
      mask(i,j) = cAB(sA[i], sB[j]);
  return mask;
}

// =====================================================================
// RA610 FK: 16 球 (4 底盤 + 12 手臂) + T_ee
// =====================================================================
// [MATLAB] 基座已含在 T_base; chain: T1=base*Tz(117), T2=Tz(448.5)*Rz(J1), ...
void GdSolver::robot_arm_bubble_RA610(const Eigen::Matrix4d& T_base, const double J[6],
                                      Eigen::MatrixXd& bubble, Eigen::VectorXd& r,
                                      Eigen::Matrix4d& T_ee)
{
  Eigen::Matrix4d T[7];
  T[0] = T_base * make_translation('z', 117.0);
  T[1] = make_translation('z', 448.5) * make_rotation('z', J[0]);
  T[2] = make_translation('y', 140.0) * make_rotation('x', J[1]);
  T[3] = make_translation('z', 640.0) * make_rotation('x', J[2]);
  T[4] = make_translation('z', 160.0) * make_rotation('y', J[3]);
  T[5] = make_translation('y', 678.0) * make_rotation('x', J[4]);
  T[6] = make_translation('y', 101.0) * make_rotation('y', J[5]);

  const int NUM_TOTAL = 16;
  const int NUM_PED   = 4;
  bubble.resize(NUM_TOTAL, 3);
  r.resize(NUM_TOTAL);

  // 底盤球: 固定在 T_base 座標系
  for (int k = 0; k < NUM_PED; ++k) {
    const BubbleDef& b = PEDESTAL_A[k];
    Eigen::Vector4d pt(b.cx, b.cy, b.cz, 1.0);
    Eigen::Vector4d w = T_base * pt;
    bubble.row(k) = w.head<3>().transpose();
    r(k) = b.radius;
  }

  // 手臂球: 按連桿累乘 T_cum 轉換
  Eigen::Matrix4d T_cum = Eigen::Matrix4d::Identity();
  int arm_k = 0;
  for (int i = 0; i < 7; ++i) {
    T_cum = T_cum * T[i];
    for (const BubbleDef& b : BUBBLES_A) {
      if (b.link_id == i) {   // [MATLAB] arm_frame(j) == i-1 (此處 0-indexed 對齊)
        Eigen::Vector4d pt(b.cx, b.cy, b.cz, 1.0);
        Eigen::Vector4d w = T_cum * pt;
        const int out_idx = NUM_PED + arm_k;
        bubble.row(out_idx) = w.head<3>().transpose();
        r(out_idx) = b.radius;
        ++arm_k;
      }
    }
  }
  T_ee = T_cum;   // 末端執行器 4x4 (7 連桿累乘)
}

// =====================================================================
// RA605 FK: 18 球 (8 底盤 + 10 手臂) + T_ee
// =====================================================================
void GdSolver::robot_arm_bubble_RA605(const Eigen::Matrix4d& T_base, const double J[6],
                                      Eigen::MatrixXd& bubble, Eigen::VectorXd& r,
                                      Eigen::Matrix4d& T_ee)
{
  Eigen::Matrix4d T[7];
  T[0] = T_base * make_translation('z', 802.0);
  T[1] = make_translation('z', 375.0) * make_rotation('z', J[0]);
  T[2] = make_translation('y',  30.0) * make_rotation('x', J[1]);
  T[3] = make_translation('z', 340.0) * make_rotation('x', J[2]);
  T[4] = make_translation('z',  40.0) * make_rotation('y', J[3]);
  T[5] = make_translation('y', 338.0) * make_rotation('x', J[4]);
  T[6] = make_translation('y',  86.5) * make_rotation('y', J[5]);

  const int NUM_TOTAL = 18;
  const int NUM_PED   = 8;
  bubble.resize(NUM_TOTAL, 3);
  r.resize(NUM_TOTAL);

  for (int k = 0; k < NUM_PED; ++k) {
    const BubbleDef& b = PEDESTAL_B[k];
    Eigen::Vector4d pt(b.cx, b.cy, b.cz, 1.0);
    Eigen::Vector4d w = T_base * pt;
    bubble.row(k) = w.head<3>().transpose();
    r(k) = b.radius;
  }

  Eigen::Matrix4d T_cum = Eigen::Matrix4d::Identity();
  int arm_k = 0;
  for (int i = 0; i < 7; ++i) {
    T_cum = T_cum * T[i];
    for (const BubbleDef& b : BUBBLES_B) {
      if (b.link_id == i) {
        Eigen::Vector4d pt(b.cx, b.cy, b.cz, 1.0);
        Eigen::Vector4d w = T_cum * pt;
        const int out_idx = NUM_PED + arm_k;
        bubble.row(out_idx) = w.head<3>().transpose();
        r(out_idx) = b.radius;
        ++arm_k;
      }
    }
  }
  T_ee = T_cum;
}

// =====================================================================
// 建構函數 (= MATLAB Dual_Arm_Inequality_ALM_GD_v6 建構函數)
// =====================================================================
GdSolver::GdSolver(const Eigen::MatrixXd& X,
                   const Eigen::Matrix4d& robotA_base,
                   const Eigen::Matrix4d& robotB_base,
                   double danger_threshold,
                   double path_weight,
                   double smooth_w,
                   double smooth_w_H,
                   double smooth_w_T,
                   double smooth_w_neighbor)
  : A_base_(robotA_base), B_base_(robotB_base),
    danger_threshold_(danger_threshold), path_weight_(path_weight),
    smooth_w_(smooth_w), smooth_w_H_(smooth_w_H),
    smooth_w_T_(smooth_w_T), smooth_w_neighbor_(smooth_w_neighbor)
{
  using std::vector;

  // [MATLAB] M = size(X,1) - 2 (扣頭尾)
  M_ = static_cast<int>(X.rows()) - 2;

  // [MATLAB] mask -> 線性索引 (column-major, 與 MATLAB reshape 一致)
  Eigen::Array<bool, 16, 18> mask = get_collision_masks();
  lin_idx_AB_.clear();
  for (int col = 0; col < 18; ++col)        // column-major: 先掃列再換行
    for (int row = 0; row < 16; ++row)
      if (mask(row, col))
        lin_idx_AB_.push_back(col * 16 + row);   // 0-indexed 線性位置
  K_AB_  = static_cast<int>(lin_idx_AB_.size());
  num_D_ = K_AB_;

  num_X_ = 2 * M_ * 6;
  num_C_ = M_ * num_D_;

  // [MATLAB] 頭尾點 X_H/X_T = [A; B]
  X_H_.row(0) = X.row(0).segment(0, 6);
  X_H_.row(1) = X.row(0).segment(6, 6);
  X_T_.row(0) = X.row(X.rows()-1).segment(0, 6);
  X_T_.row(1) = X.row(X.rows()-1).segment(6, 6);

  // [MATLAB] 中間點原始關節角 oriPos = [Xa_ori, Xb_ori] (M x 12)
  oriPos_.resize(M_, 12);
  for (int m = 0; m < M_; ++m)
    oriPos_.row(m) = X.row(m+1);   // 第 2..end-1 列

  // [MATLAB] 初始決策向量 X_vec: 每點 [A1..6, B1..6]
  Xm_initial_.resize(num_X_);
  for (int m = 0; m < M_; ++m) {
    const int b = base_x(m);
    for (int j = 0; j < 6; ++j) {
      Xm_initial_(b + j)     = oriPos_(m, j);       // A
      Xm_initial_(b + 6 + j) = oriPos_(m, 6 + j);   // B
    }
  }

  // [MATLAB] ALM 初值
  mu_ = Eigen::VectorXd::Constant(num_C_, 10.0);   // mu_0 = 10
  c_  = 5.0;

  X_final_ = Xm_initial_;
}

// =====================================================================
// compute_Dm: 第 m 點 (0-indexed) 的危險因子向量 (num_D)
// =====================================================================
// [MATLAB] FK -> calc_df -> 用 lin_idx_AB 過濾出 mask 內的球對
Eigen::VectorXd GdSolver::compute_Dm(const Eigen::VectorXd& X, int m) const
{
  const int b = base_x(m);
  double Ja[6], Jb[6];
  for (int j = 0; j < 6; ++j) { Ja[j] = X(b + j); Jb[j] = X(b + 6 + j); }

  Eigen::MatrixXd bA, bB; Eigen::VectorXd rA, rB; Eigen::Matrix4d eeA, eeB;
  robot_arm_bubble_RA610(A_base_, Ja, bA, rA, eeA);
  robot_arm_bubble_RA605(B_base_, Jb, bB, rB, eeB);

  Eigen::MatrixXd sj = calc_df(rA, rB, bA, bB);   // 16x18

  // [MATLAB] sj_flat = reshape(sj,288,1) (column-major), 取 lin_idx_AB
  Eigen::VectorXd D_m(K_AB_);
  for (int k = 0; k < K_AB_; ++k) {
    const int idx = lin_idx_AB_[k];
    const int row = idx % 16;
    const int col = idx / 16;
    D_m(k) = sj(row, col);
  }
  return D_m;
}

// [MATLAB] compute_Dx_all: 全 M 點串接 (num_C)
Eigen::VectorXd GdSolver::compute_Dx_all(const Eigen::VectorXd& X) const
{
  Eigen::VectorXd Dx(num_C_);
  for (int m = 0; m < M_; ++m)
    Dx.segment(m * num_D_, num_D_) = compute_Dm(X, m);
  return Dx;
}

// =====================================================================
// compute_D_cache: D_base + D_plus (前向差分用, GD 不需 D_minus)
// =====================================================================
// [MATLAB] D_base:(num_D x M), D_plus:(num_D x 12 x M) 用 vector<MatrixXd>
void GdSolver::compute_D_cache(const Eigen::VectorXd& X,
                               Eigen::MatrixXd& D_base,
                               std::vector<Eigen::MatrixXd>& D_plus) const
{
  const double h = delta_;
  D_base.resize(num_D_, M_);
  D_plus.assign(M_, Eigen::MatrixXd(num_D_, 12));

  for (int m = 0; m < M_; ++m) {
    D_base.col(m) = compute_Dm(X, m);
    const int b = base_x(m);
    for (int i = 0; i < 12; ++i) {
      Eigen::VectorXd Xp = X;
      Xp(b + i) += h;
      D_plus[m].col(i) = compute_Dm(Xp, m);
    }
  }
}

// =====================================================================
// compute_G_smooth: 平滑項梯度 (前向差分)
// =====================================================================
Eigen::VectorXd GdSolver::compute_G_smooth(const Eigen::VectorXd& X) const
{
  const double h = delta_;
  const double f0 = cost_function_F(X);
  Eigen::VectorXd G(num_X_);
  for (int i = 0; i < num_X_; ++i) {
    Eigen::VectorXd Xp = X;
    Xp(i) += h;
    G(i) = (cost_function_F(Xp) - f0) / h;
  }
  return G;
}

// =====================================================================
// compute_G_c: 完整梯度 grad_X L_rho = grad f + sum t_i * grad g_i
// =====================================================================
// [MATLAB] t_i = max(0, mu_i + c*g_i); grad g_i = (D_plus - D_base)/h (block-diagonal)
Eigen::VectorXd GdSolver::compute_G_c(const Eigen::VectorXd& X,
                                      const Eigen::MatrixXd& D_base,
                                      const std::vector<Eigen::MatrixXd>& D_plus,
                                      const Eigen::VectorXd& mu_loc, double c_loc) const
{
  const double h = delta_;
  Eigen::VectorXd G = compute_G_smooth(X);   // 起手為平滑梯度

  for (int m = 0; m < M_; ++m) {
    const int bx = base_x(m);
    const int i0 = m * num_D_;
    Eigen::VectorXd g_m  = D_base.col(m).array() - danger_threshold_;
    Eigen::VectorXd mu_m = mu_loc.segment(i0, num_D_);
    // t_m = max(0, mu_m + c*g_m)
    Eigen::VectorXd t_m = (mu_m + c_loc * g_m).cwiseMax(0.0);
    for (int j = 0; j < 12; ++j) {
      Eigen::VectorXd J_col = (D_plus[m].col(j) - D_base.col(m)) / h;
      G(bx + j) += t_m.dot(J_col);
    }
  }
  return G;
}

// =====================================================================
// cost_Xm: 單臂平滑成本 (距原始 + 頭尾 + 鄰點)
// =====================================================================
double GdSolver::cost_Xm(const Eigen::MatrixXd& Xa, const Eigen::MatrixXd& Xori,
                         const Eigen::RowVectorXd& XH, const Eigen::RowVectorXd& XT) const
{
  const int Md = static_cast<int>(Xa.rows());
  double c = 0.0;
  for (int m = 0; m < Md; ++m) {
    Eigen::RowVectorXd dv = Xa.row(m) - Xori.row(m);
    c += smooth_w_ * dv.dot(dv);
  }
  Eigen::RowVectorXd dH = Xa.row(0) - XH;
  c += smooth_w_H_ * dH.dot(dH);
  for (int m = 1; m < Md; ++m) {
    Eigen::RowVectorXd dv = Xa.row(m) - Xa.row(m-1);
    c += smooth_w_neighbor_ * dv.dot(dv);
  }
  Eigen::RowVectorXd dT = Xa.row(Md-1) - XT;
  c += smooth_w_T_ * dT.dot(dT);
  return c;
}

// [MATLAB] cost_function_F: f = pw*fA + (1-pw)*fB
double GdSolver::cost_function_F(const Eigen::VectorXd& X) const
{
  double f, fa, fb;
  cost_function_F_split(X, f, fa, fb);
  return f;
}

void GdSolver::cost_function_F_split(const Eigen::VectorXd& X,
                                     double& f, double& fa, double& fb) const
{
  Eigen::MatrixXd Xa(M_, 6), Xb(M_, 6);
  for (int m = 0; m < M_; ++m) {
    const int b = base_x(m);
    for (int j = 0; j < 6; ++j) { Xa(m,j) = X(b+j); Xb(m,j) = X(b+6+j); }
  }
  fa = cost_Xm(Xa, oriPos_.leftCols(6),  X_H_.row(0), X_T_.row(0));
  fb = cost_Xm(Xb, oriPos_.rightCols(6), X_H_.row(1), X_T_.row(1));
  f  = path_weight_ * fa + (1.0 - path_weight_) * fb;
}

// =====================================================================
// cost_L_loc: Augmented Lagrangian L = f + (1/2c)*sum{[max(0,mu+c*g)]^2 - mu^2}
// =====================================================================
double GdSolver::cost_L_loc(const Eigen::VectorXd& X,
                            const Eigen::VectorXd& mu_loc, double c_loc) const
{
  Eigen::VectorXd Dx = compute_Dx_all(X);
  Eigen::VectorXd g  = Dx.array() - danger_threshold_;
  const double f = cost_function_F(X);
  Eigen::VectorXd t = (mu_loc + c_loc * g).cwiseMax(0.0);
  const double pen = (1.0 / (2.0 * c_loc)) *
                     (t.array().square() - mu_loc.array().square()).sum();
  return f + pen;
}

// =====================================================================
// line_search_newton_1d: 1D Newton 線搜索 phi(a)=L_rho(X+a*d)
// =====================================================================
// [MATLAB] a <- a - phi'/phi''; phi',phi'' 3 點中心差分; 失敗退 fallback
double GdSolver::line_search_newton_1d(const Eigen::VectorXd& X, const Eigen::VectorXd& d,
                                       const Eigen::VectorXd& mu_loc, double c_loc) const
{
  const double LS_DELTA = 0.001;
  const int    MAX_INNER = 50;
  const double H_MIN_ABS = 1e-9;
  const double ANEW_TOL  = 1e-3;
  const double FALLBACK_ALPHA = 0.0001;
  const int    FAIL_CHECK_IT  = 10;

  double a = 0.0;
  for (int k = 1; k <= MAX_INNER; ++k) {
    const double f0 = cost_L_loc(X + a            * d, mu_loc, c_loc);
    const double fp = cost_L_loc(X + (a+LS_DELTA) * d, mu_loc, c_loc);
    const double fm = cost_L_loc(X + (a-LS_DELTA) * d, mu_loc, c_loc);

    const double g_phi = (fp - fm) / (2.0 * LS_DELTA);
    const double h_phi = (fp - 2.0*f0 + fm) / (LS_DELTA * LS_DELTA);

    if (std::abs(h_phi) < H_MIN_ABS) return FALLBACK_ALPHA;

    const double anew = a - g_phi / h_phi;
    if (std::abs(anew - a) < ANEW_TOL) { a = anew; break; }
    a = anew;
    if (k > FAIL_CHECK_IT && (std::isnan(a) || std::isinf(a))) return FALLBACK_ALPHA;
  }
  return a;
}

// =====================================================================
// run_alm: ALM 外層 + GD 內層 + 線搜索 (= MATLAB run_alm)
// =====================================================================
SolverLog GdSolver::run_alm()
{
  using std::vector;
  SolverLog log;

  Eigen::VectorXd mu_loc = mu_;
  double c_loc  = c_;
  double eps_in = epsilon_inner_;

  Eigen::VectorXd X = Xm_initial_;
  double v_prev = std::numeric_limits<double>::infinity();
  double G_norm = 0.0, d_norm = 0.0, d_last_outer = 0.0;

  int outer_cnt = 0, inner_cnt = 0;

  for (int k_outer = 1; k_outer <= K_outer_; ++k_outer) {
    const int K_inner_now = (k_outer == 1) ? K_inner_first_ : K_inner_;
    int inner_done = 0;

    // ===== [Step 1] 內層 min_X L_rho =====
    for (int inner = 1; inner <= K_inner_now; ++inner) {
      // (1a) FK 擾動快取
      Eigen::MatrixXd D_base; vector<Eigen::MatrixXd> D_plus;
      compute_D_cache(X, D_base, D_plus);

      // (1b) 梯度
      Eigen::VectorXd G = compute_G_c(X, D_base, D_plus, mu_loc, c_loc);
      const double G_norm2 = G.squaredNorm();
      G_norm = std::sqrt(G_norm2);

      // (1c) 發散偵測 (梯度)
      if (!G.allFinite() || G_norm > 1e10) {
        std::cout << "  * 內層發散! outer=" << k_outer << " inner=" << inner
                  << " ||G||=" << G_norm << "\n";
        log.diverge_iter = k_outer;
        break;
      }

      // (1d) GD 方向: 最陡下降 d = -G  (= MATLAB GD_v6)
      //   [vs CG] GD 無方向記憶; CG 版的 FR 動量簿記 (pre_G_norm2/pre_d) 已移除
      Eigen::VectorXd d = -G;

      // (1e) 1D Newton 線搜索
      const double a = line_search_newton_1d(X, d, mu_loc, c_loc);

      // (1f) 更新 X
      X = X + a * d;

      d_norm = d.norm();
      ++inner_cnt;
      log.alpha_history.push_back(a);
      log.G_norm_history.push_back(G_norm);
      log.d_norm_history.push_back(d_norm);

      // (1h) 內層收斂
      if (G_norm < eps_in) { inner_done = inner; break; }

    }  // inner

    d_last_outer = d_norm;
    if (log.diverge_iter > 0) break;
    if (inner_done == 0) inner_done = K_inner_now;

    // ===== 外層: 計算 D / 成本 / KKT 度量 (用更新前 mu_loc) =====
    Eigen::VectorXd Dx_all = compute_Dx_all(X);
    Eigen::VectorXd g_all  = Dx_all.array() - danger_threshold_;
    const double max_D = Dx_all.maxCoeff();
    double f_curr, fa_curr, fb_curr;
    cost_function_F_split(X, f_curr, fa_curr, fb_curr);

    Eigen::VectorXd t_all_L = (mu_loc + c_loc * g_all).cwiseMax(0.0);
    const double pen_curr = (1.0/(2.0*c_loc)) *
                            (t_all_L.array().square() - mu_loc.array().square()).sum();
    const double L_curr = f_curr + pen_curr;

    double v_pure = 0.0, compl_v = 0.0, v_curr = 0.0;
    for (int ii = 0; ii < num_C_; ++ii) {
      const double g_i  = g_all(ii);
      const double mu_i = mu_loc(ii);
      const double cp_i = std::min(-g_i, mu_i);
      const double vk_i = std::min(-g_i, mu_i / c_loc);
      if (g_i  > v_pure)  v_pure  = g_i;
      if (cp_i > compl_v) compl_v = cp_i;
      if (std::abs(vk_i) > v_curr) v_curr = std::abs(vk_i);
    }

    // ===== [Step 3] 乘子更新 mu = max(0, mu + c*g)（古典 PHR，Bertsekas 1982, eq.17）=====
    for (int ii = 0; ii < num_C_; ++ii) {
      double val = mu_loc(ii) + c_loc * g_all(ii);
      if (val < 0.0) val = 0.0;
      mu_loc(ii) = val;
    }

    // ===== [Step 2] 罰參數更新 c <- beta_c*c 若 v_curr > gamma_v*v_prev =====
    bool c_updated = false;
    // if (v_pure < epsilon_v_) {
    //   c_updated = false;
    // } else if (k_outer > 1 && v_curr > gamma_v_ * v_prev) {
    //   if (c_loc < c_max_) { c_loc = std::min(c_loc * beta_c_, c_max_); c_updated = true; }
    // }
    if (k_outer > 1 && v_curr > gamma_v_ * v_prev) {
      if (c_loc < c_max_) { c_loc = std::min(c_loc * beta_c_, c_max_); c_updated = true; }
    }
    v_prev = v_curr;

    // 內層精度遞減
    eps_in = std::max(epsilon_inner_min_, epsilon_inner_decay_ * eps_in);

    ++outer_cnt;
    const double mu_max_cur = mu_loc.maxCoeff();
    // 填外層歷史
    log.L_history.push_back(L_curr);
    log.f_history_f.push_back(f_curr);
    log.f_history_fa.push_back(fa_curr);
    log.f_history_fb.push_back(fb_curr);
    log.penality_history.push_back(pen_curr);
    log.maxD_history.push_back(max_D);
    log.c_history.push_back(c_loc);
    log.mu_max_history.push_back(mu_max_cur);
    log.vk_history.push_back(v_curr);
    log.v_pure_history.push_back(v_pure);
    log.compl_history.push_back(compl_v);
    log.G_last_history.push_back(G_norm);
    log.d_last_history.push_back(d_last_outer);
    log.inner_iter.push_back(inner_done);

    std::cout << "  [Outer " << k_outer << "] f:" << f_curr
              << " | MaxD:" << max_D << " | v_pure:" << v_pure
              << " | V_k:" << v_curr << " | compl:" << compl_v
              << " | ||G||:" << G_norm << " | inner:" << inner_done
              << "/" << K_inner_now << " | c" << (c_updated ? "(up)" : "") << ":"
              << c_loc << " | eps_in:" << eps_in << "\n";

    // ===== 停止: 三條 KKT =====
    if (v_pure < epsilon_v_ && G_norm < epsilon_g_ && compl_v < epsilon_compl_) {
      std::cout << "\n  * ALM 收斂! (3-KKT) outer=" << k_outer << "\n";
      log.outer_break_iter = k_outer;
      break;
    }
  }

  X_final_ = X;

  if (log.outer_break_iter < 0 && log.diverge_iter < 0 && c_loc >= c_max_)
    log.feasibility_mode = true;

  log.outer_iter   = outer_cnt;
  log.total_inner  = inner_cnt;
  log.mu_final     = mu_loc;
  log.c_final      = c_loc;
  log.final_G_norm = G_norm;
  log.final_D      = compute_Dx_all(X_final_);

  std::cout << "\n========== ALM 結果摘要 ==========\n"
            << "  外層迭代: " << outer_cnt << " | 內層總迭代: " << inner_cnt << "\n";
  if (log.outer_break_iter > 0)
    std::cout << "  狀態: 收斂於 iter " << log.outer_break_iter << " (3-KKT)\n";
  else if (log.diverge_iter > 0)
    std::cout << "  狀態: 發散 (outer=" << log.diverge_iter << ")\n";
  else if (log.feasibility_mode)
    std::cout << "  狀態: c 達上限 fallback\n";
  else
    std::cout << "  狀態: 達迭代上限未收斂\n";
  std::cout << "===================================\n\n";

  return log;
}

}  // namespace dual_arm_alm_gd_planner
