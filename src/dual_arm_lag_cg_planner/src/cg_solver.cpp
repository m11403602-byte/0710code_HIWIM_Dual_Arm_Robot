// =====================================================================
// cg_solver.cpp — 第 1 層 純 Lagrangian 共軛梯度求解器實作
//   = MATLAB Dual_Arm_Lagrangian_Gradient_v2
// =====================================================================
//   幾何層 (球常數 / transmatrix / calc_df / mask / FK) 與 ALM、Newton
//   譜系位元一致 (同一套機器人模型, 已逐項對照 v2 MATLAB 驗證)。
//   solver 數學層 (L / G / line_search / run_lag) 照 Con_v2 移植。
//
//   本檔只被 avoidance_system.cpp 的 run_solver_global() 用到（每輪外層
//   修復迴圈都會 new 一個新的 CgSolver）。檔案內容分兩塊：
//     (a) 純數學/幾何 static 工具（不依賴物件狀態，任何人都能呼叫）
//     (b) CgSolver 物件本身的建構與優化流程
//
//   ★ 完整呼叫順序（一次 CgSolver 生命週期）★
//   ---------------------------------------------------------------
//   0. [靜態工具, 誰都能呼叫, 不需要先 new CgSolver]
//        make_rotation / make_translation      (L~69, L~84)  FK 用的基本矩陣
//        calc_df                                (L~96)        球對危險因子公式
//        get_collision_masks                    (L~119)       16x18 跨臂 mask
//        robot_arm_bubble_RA610 / RA605          (L~136, L~180) 正向運動學 + 包覆球
//
//   1. 建構子 CgSolver(X, ...)                    (L~224)
//        └─ rebuild_initial_V_()                  (L~283)
//             → 組出決策變數初值 V0 = [X0; λ0; S0]
//        （此時尚未優化，只是把資料準備好）
//
//   2. avoidance_system.cpp 呼叫 set_lag_params() → 也會觸發
//      rebuild_initial_V_() 重建一次 V0（用新參數覆蓋建構子的預設值）
//
//   3. avoidance_system.cpp 呼叫 run_lag()          (L~530)  ← 主迴圈
//        每一次迭代 it = 1..max_solver_iter_ 依序:
//        (1) compute_D_cache(Xn)                    (L~341)
//              └─ 對每個中間點呼叫 compute_Dm()       (L~307)
//                   └─ robot_arm_bubble_RA610/RA605 + calc_df (步驟 0 的工具)
//              → 順便用有限差分算出 D 對每個關節的敏感度 (D_plus)
//        (2) compute_G(Xn, D_base, D_plus)           (L~385)  KKT 殘差 G
//              └─ compute_G_smooth()                  (L~364)
//                   └─ cost_function_F()                (L~436)
//                        └─ cost_function_F_split()       (L~444)
//                             └─ cost_Xm() (A臂, B臂各一次)  (L~462)
//        (3) CG 方向 d = -G 或 Fletcher-Reeves 更新 (主迴圈內聯, 非獨立函式)
//        (4) line_search_newton_1d(Xn, d)             (L~491)  找步長 α
//              └─ 迴圈內反覆呼叫 cost_function_L()      (L~421)
//                   └─ cost_function_F() + compute_Dx_all() (L~329)
//                        └─ compute_Dm() (對每個中間點)
//        (5) Xn ← Xn + α·d，記錄本輪各項殘差/成本到 SolverLog
//        (6) 收斂判定 (phys_ok && stable_ok) → 是就 break，回傳 SolverLog
//   ---------------------------------------------------------------
//   數學符號對照、KKT 推導細節見 CODE_WALKTHROUGH.md。
// =====================================================================
#include "dual_arm_lag_cg_planner/cg_solver.hpp"

#include <cmath>
#include <iostream>
#include <algorithm>
#include <limits>

namespace dual_arm_lag_cg_planner
{

// =====================================================================
// 包覆球常數 (= MATLAB robot_arm_bubble 內 arm_r/arm_p/arm_frame、ped_*)
//   ⚠ 與 ALM/Newton 譜系位元一致 — 換機器人時六包需同步修改
// =====================================================================
const std::vector<BubbleDef> CgSolver::PEDESTAL_A = {
  { -1, 320.0,  220.0,  -65.0, 55.0 },
  { -1, 320.0,  220.0,  296.0, 55.0 },
  { -1, 320.0, -220.0,  296.0, 55.0 },
  { -1, 320.0, -220.0,  -64.0, 55.0 },
};
const std::vector<BubbleDef> CgSolver::BUBBLES_A = {
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
const std::vector<BubbleDef> CgSolver::PEDESTAL_B = {
  { -1, 350.0,  170.0,  290.0, 661.0 },
  { -1, 350.0, -170.0,  290.0, 661.0 },
  { -1, 350.0, -170.0, -230.0, 661.0 },
  { -1, 350.0,  170.0, -230.0, 661.0 },
  { -1, 350.0,  185.0,  250.0, 231.0 },
  { -1, 350.0, -185.0,  250.0, 231.0 },
  { -1, 350.0, -185.0, -250.0, 231.0 },
  { -1, 350.0,  185.0, -250.0, 231.0 },
};
const std::vector<BubbleDef> CgSolver::BUBBLES_B = {
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
// 【呼叫順序 0】transmatrix → make_rotation / make_translation
//   最底層的基本工具，被下面 robot_arm_bubble_RA610/RA605 疊乘組出
//   完整正向運動學鏈；沒有任何函式依賴它們，可獨立測試
//   (= MATLAB transmatrix)
// =====================================================================
Eigen::Matrix4d CgSolver::make_rotation(char axis, double angle_deg)
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

Eigen::Matrix4d CgSolver::make_translation(char axis, double dist_mm)
{
  Eigen::Matrix4d m = Eigen::Matrix4d::Identity();
  if (axis == 'x' || axis == 'X')      m(0,3) = dist_mm;
  else if (axis == 'y' || axis == 'Y') m(1,3) = dist_mm;
  else if (axis == 'z' || axis == 'Z') m(2,3) = dist_mm;
  return m;
}

// =====================================================================
// 【呼叫順序 0】calc_df — 被 compute_Dm() 呼叫，算「一組球 vs 另一組球」
//   兩兩之間的危險因子矩陣；本身不知道機器人，純幾何距離公式
//   球對危險因子 sj = exp( ln(0.5)/(R_i+R_j)^2 · d^2 )
// =====================================================================
Eigen::MatrixXd CgSolver::calc_df(const Eigen::VectorXd& R1, const Eigen::VectorXd& R2,
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
// 【呼叫順序 0】get_collision_masks — 被建構子 (一次) 與 check_collision()
//   的內部 lambda (每個時間步一次) 呼叫，決定 16x18 顆球中「哪些球對」
//   才需要真的檢查距離（例如兩臂底座附近的球，物理上不可能碰到，
//   直接跳過省算力）
//   16x18 跨臂 mask (連桿級 cAB 展開到球級, K_AB=180)
// =====================================================================
Eigen::Array<bool, 16, 18> CgSolver::get_collision_masks()
{
  // 球 → 連桿 ID (1-indexed, 同 MATLAB sA/sB)
  static const int sA[16] = {1,1,1,1, 2, 3, 4,4,4,4, 5, 6,6,6, 7, 8};
  static const int sB[18] = {1,1,1,1,1,1,1,1, 2, 3, 4,4,4, 5, 6,6, 7, 8};
  // [MATLAB] cAB: 連桿 1~3 (底盤/基座/L1) 全 0; 連桿 4~8 全 1
  auto cAB = [](int rowLink, int /*colLink*/) -> bool { return rowLink >= 4; };
  Eigen::Array<bool, 16, 18> mask;
  for (int i = 0; i < 16; ++i)
    for (int j = 0; j < 18; ++j)
      mask(i,j) = cAB(sA[i], sB[j]);
  return mask;
}

// =====================================================================
// 【呼叫順序 0】RA610 FK — 被 compute_Dm()（內層每次要算 D 都要重算一次
//   FK）與 avoidance_system.cpp 的 check_collision() 呼叫
//   輸入 6 個關節角 J[6]，用 make_translation/make_rotation 疊乘出整條
//   運動學鏈，再把每顆包覆球從「連桿局部座標」轉到「世界座標」
//   16 球 (4 底盤 + 12 手臂) + T_ee
// =====================================================================
void CgSolver::robot_arm_bubble_RA610(const Eigen::Matrix4d& T_base, const double J[6],
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

  const int NUM_TOTAL = 16, NUM_PED = 4;
  bubble.resize(NUM_TOTAL, 3);
  r.resize(NUM_TOTAL);

  for (int k = 0; k < NUM_PED; ++k) {
    const BubbleDef& b = PEDESTAL_A[k];
    Eigen::Vector4d w = T_base * Eigen::Vector4d(b.cx, b.cy, b.cz, 1.0);
    bubble.row(k) = w.head<3>().transpose();
    r(k) = b.radius;
  }

  Eigen::Matrix4d T_cum = Eigen::Matrix4d::Identity();
  int arm_k = 0;
  for (int i = 0; i < 7; ++i) {
    T_cum = T_cum * T[i];
    for (const BubbleDef& b : BUBBLES_A) {
      if (b.link_id == i) {   // [MATLAB] arm_frame(j) == i-1 (0-indexed 對齊)
        Eigen::Vector4d w = T_cum * Eigen::Vector4d(b.cx, b.cy, b.cz, 1.0);
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
// 【呼叫順序 0】RA605 FK — 與上面 RA610 FK 對稱，同樣被 compute_Dm() /
//   check_collision() 呼叫，只是換一組連桿長度與球表 (B 臂專用)
//   18 球 (8 底盤 + 10 手臂) + T_ee
// =====================================================================
void CgSolver::robot_arm_bubble_RA605(const Eigen::Matrix4d& T_base, const double J[6],
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

  const int NUM_TOTAL = 18, NUM_PED = 8;
  bubble.resize(NUM_TOTAL, 3);
  r.resize(NUM_TOTAL);

  for (int k = 0; k < NUM_PED; ++k) {
    const BubbleDef& b = PEDESTAL_B[k];
    Eigen::Vector4d w = T_base * Eigen::Vector4d(b.cx, b.cy, b.cz, 1.0);
    bubble.row(k) = w.head<3>().transpose();
    r(k) = b.radius;
  }

  Eigen::Matrix4d T_cum = Eigen::Matrix4d::Identity();
  int arm_k = 0;
  for (int i = 0; i < 7; ++i) {
    T_cum = T_cum * T[i];
    for (const BubbleDef& b : BUBBLES_B) {
      if (b.link_id == i) {
        Eigen::Vector4d w = T_cum * Eigen::Vector4d(b.cx, b.cy, b.cz, 1.0);
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
// 【呼叫順序 1】建構函數 — 由 avoidance_system.cpp 的 run_solver_global()
//   每輪外層修復都 new 一個新的 CgSolver (不重複使用舊物件)
//   做的事：算出決策變數維度 (M_/num_D_/num_X_/num_C_/total_dim_)、
//   記住頭尾兩個不動的邊界點 (X_H_/X_T_)、記住中間點的原始角度做平滑
//   參考 (oriPos_)，最後呼叫 rebuild_initial_V_() 組出優化起始值 V0
//   (= MATLAB Dual_Arm_Lagrangian_Gradient_v2 建構函數)
// =====================================================================
CgSolver::CgSolver(const Eigen::MatrixXd& X,
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
  // [MATLAB] M = size(X,1) - 2 (扣頭尾); N = 6
  M_ = static_cast<int>(X.rows()) - 2;
  N_ = 6;

  // [MATLAB] mask → 線性索引 (column-major, 與 MATLAB logical 索引一致)
  Eigen::Array<bool, 16, 18> mask = get_collision_masks();
  lin_idx_AB_.clear();
  for (int col = 0; col < 18; ++col)
    for (int row = 0; row < 16; ++row)
      if (mask(row, col))
        lin_idx_AB_.push_back(col * 16 + row);
  K_AB_  = static_cast<int>(lin_idx_AB_.size());
  num_D_ = K_AB_;

  num_X_     = 2 * M_ * N_;
  num_C_     = M_ * num_D_;
  total_dim_ = num_X_ + 2 * num_C_;

  std::cout << "  [Mask] A×B: " << K_AB_ << " → num_D = " << num_D_ << "\n";

  // [MATLAB] 頭尾點 X_H/X_T = [A; B]
  X_H_.row(0) = X.row(0).segment(0, 6);
  X_H_.row(1) = X.row(0).segment(6, 6);
  X_T_.row(0) = X.row(X.rows()-1).segment(0, 6);
  X_T_.row(1) = X.row(X.rows()-1).segment(6, 6);

  // [MATLAB] 中間點原始關節角 oriPos = [Xa_ori, Xb_ori] (M x 12)
  oriPos_.resize(M_, 12);
  for (int m = 0; m < M_; ++m)
    oriPos_.row(m) = X.row(m + 1);

  // [MATLAB] V_0 = [X_0; λ_0; S_0]; X_0 每點 [A1..6, B1..6]
  rebuild_initial_V_();

  std::cout << "  [Init] Max_D=" << compute_Dx_all(Xm_initial_.head(num_X_)).maxCoeff()
            << " | Method=CG+1D-Newton (pure Lagrangian)\n";

  V_final_ = Xm_initial_;
  X_final_ = Xm_initial_.head(num_X_);
}

// =====================================================================
// 【呼叫順序 1a / 2】rebuild_initial_V_ — 被兩處呼叫:
//   (1) 建構子最後一步 (用預設 lam0_/s0_)
//   (2) set_lag_params() 被 avoidance_system 呼叫時 (用 yaml 真正的值
//       覆蓋掉建構子預設值，重新組一次 V0)
//   run_lag() 開始迭代前，V0 一定是這個函式組出來的最新版本
//   以當前 oriPos / lam0_ / s0_ 重建 V_0
// =====================================================================
void CgSolver::rebuild_initial_V_()
{
  Xm_initial_.resize(total_dim_);

  // X 段 (num_X): 每點 [A;B]
  for (int m = 0; m < M_; ++m) {
    const int b = idx_Xm(m);
    for (int j = 0; j < 6; ++j) {
      Xm_initial_(b + j)     = oriPos_(m, j);       // A
      Xm_initial_(b + 6 + j) = oriPos_(m, 6 + j);   // B
    }
  }

  // λ 段 (num_C): 全 lam0_
  //   [MATLAB] lam_init(Dx_init>θ)=30 為 future ablation 入口; 因 lam0=30 等價全填
  Xm_initial_.segment(num_X_, num_C_).setConstant(lam0_);
  // S 段 (num_C): 全 s0_ (S² = s0_²)
  Xm_initial_.segment(num_X_ + num_C_, num_C_).setConstant(s0_);
}

// =====================================================================
// 【呼叫順序 3-1】compute_Dm — run_lag() 主迴圈裡最常被呼叫的函式
//   （直接被 compute_D_cache()/compute_Dx_all() 呼叫，
//    間接透過它們被 run_lag() 每次迭代呼叫非常多次）
//   給第 m 個中間點的 12 個關節角，做一次 FK (RA610+RA605) + calc_df，
//   回傳這一點的危險因子向量 (num_D 維)
//   第 m 點 (0-indexed) 危險因子向量 (num_D)
//   X: num_X 維 X 向量 (非完整 V)
// =====================================================================
Eigen::VectorXd CgSolver::compute_Dm(const Eigen::VectorXd& X, int m) const
{
  double Ja[6], Jb[6];
  const int ba = idx_Xam(m), bb = idx_Xbm(m);
  for (int j = 0; j < 6; ++j) { Ja[j] = X(ba + j); Jb[j] = X(bb + j); }

  Eigen::MatrixXd bA, bB; Eigen::VectorXd rA, rB; Eigen::Matrix4d eeA, eeB;
  robot_arm_bubble_RA610(A_base_, Ja, bA, rA, eeA);
  robot_arm_bubble_RA605(B_base_, Jb, bB, rB, eeB);

  Eigen::MatrixXd sj = calc_df(rA, rB, bA, bB);   // 16x18

  // [MATLAB] D_m = sj(mask_AB) → column-major 取 lin_idx_AB
  Eigen::VectorXd D_m(K_AB_);
  for (int k = 0; k < K_AB_; ++k) {
    const int idx = lin_idx_AB_[k];
    D_m(k) = sj(idx % 16, idx / 16);
  }
  return D_m;
}

// 【呼叫順序 3-1'】compute_Dx_all — 被 cost_function_L() 呼叫（線搜索
//   評估目標函數時要拿全部約束值），對每個中間點呼叫一次 compute_Dm()
//   後串接起來
// [MATLAB] compute_Dx_all: 全 M 點串接 (num_C)
Eigen::VectorXd CgSolver::compute_Dx_all(const Eigen::VectorXd& X) const
{
  Eigen::VectorXd Dx(num_C_);
  for (int m = 0; m < M_; ++m)
    Dx.segment(idx_lam_local(m), num_D_) = compute_Dm(X, m);
  return Dx;
}

// =====================================================================
// 【呼叫順序 3-1a】compute_D_cache — run_lag() 主迴圈「每次迭代第一步」
//   呼叫；對每個中間點呼叫 compute_Dm() 算「現在的 D」(D_base)，
//   再對每個關節做一次 +h 有限差分擾動、各呼叫一次 compute_Dm() 算出
//   「D 對這個關節的敏感度」(D_plus)，這批快取資料會被下一步
//   compute_G() 拿去算梯度，避免重複呼叫 FK
//   D_base (num_D x M) + D_plus (M 個 num_D x 12)
//   前向差分 (CG/GD 不需 D_minus); 對應 MATLAB compute_D_cache
// =====================================================================
void CgSolver::compute_D_cache(const Eigen::VectorXd& V,
                               Eigen::MatrixXd& D_base,
                               std::vector<Eigen::MatrixXd>& D_plus) const
{
  const double h = delta_;
  const Eigen::VectorXd X = V.head(num_X_);
  D_base.resize(num_D_, M_);
  D_plus.assign(M_, Eigen::MatrixXd(num_D_, 12));

  for (int m = 0; m < M_; ++m) {
    D_base.col(m) = compute_Dm(X, m);
    const int b = idx_Xm(m);
    for (int i = 0; i < 12; ++i) {
      Eigen::VectorXd Xp = X;
      Xp(b + i) += h;
      D_plus[m].col(i) = compute_Dm(Xp, m);
    }
  }
}

// =====================================================================
// 【呼叫順序 3-2a】compute_G_smooth — 被 compute_G() 呼叫，算平滑成本 f
//   對每個關節的梯度 (有限差分)；每個關節都要呼叫一次 cost_function_F()
//   ∇_X f (前向差分; f 僅依賴 X)
// =====================================================================
Eigen::VectorXd CgSolver::compute_G_smooth(const Eigen::VectorXd& V) const
{
  const double h = delta_;
  const double f0 = cost_function_F(V);
  Eigen::VectorXd G(num_X_);
  Eigen::VectorXd Vp = V;
  for (int i = 0; i < num_X_; ++i) {
    const double orig = Vp(i);
    Vp(i) = orig + h;
    G(i) = (cost_function_F(Vp) - f0) / h;
    Vp(i) = orig;
  }
  return G;
}

// =====================================================================
// 【呼叫順序 3-2】compute_G — run_lag() 主迴圈「每次迭代第二步」呼叫
//   用 compute_D_cache() 給的 D_base/D_plus (不用重算 FK) + 呼叫
//   compute_G_smooth() 算出的 ∇f，組合出完整 KKT 一階殘差 G，這是
//   後面 CG 搜索方向 d = -G 的直接輸入
//   完整 KKT 一階殘差 G = [G_X; G_λ; G_S]  (total_dim)
//   G_X = ∇f + w_d Σ λ_i ∇D_i        (stationarity)
//   G_λ = w_d · (D − θ + S²)          (primal feasibility)
//   G_S = 2 w_d · S ⊙ λ              (complementarity)
// =====================================================================
Eigen::VectorXd CgSolver::compute_G(const Eigen::VectorXd& V,
                                    const Eigen::MatrixXd& D_base,
                                    const std::vector<Eigen::MatrixXd>& D_plus) const
{
  const double h = delta_;
  const Eigen::VectorXd lambda = V.segment(num_X_, num_C_);
  const Eigen::VectorXd S      = V.segment(num_X_ + num_C_, num_C_);

  // G_X = G_smooth + G_collision
  Eigen::VectorXd G_X = compute_G_smooth(V);
  Eigen::VectorXd D_all(num_C_);
  for (int m = 0; m < M_; ++m) {
    const int i0 = idx_lam_local(m);
    const Eigen::VectorXd lam_m = lambda.segment(i0, num_D_);
    D_all.segment(i0, num_D_) = D_base.col(m);
    const double val_base = lam_m.dot(D_base.col(m)) * wd_;
    const int bx = idx_Xm(m);
    for (int j = 0; j < 12; ++j) {
      const double val_new = lam_m.dot(D_plus[m].col(j)) * wd_;
      G_X(bx + j) += (val_new - val_base) / h;
    }
  }

  // G_λ = w_d (D − θ + S²);  G_S = 2 w_d S⊙λ
  Eigen::VectorXd G_lam = (D_all.array() - danger_threshold_ + S.array().square()).matrix() * wd_;
  Eigen::VectorXd G_S   = 2.0 * wd_ * (S.array() * lambda.array()).matrix();

  Eigen::VectorXd G(total_dim_);
  G << G_X, G_lam, G_S;
  return G;
}

// =====================================================================
// 【呼叫順序 4-內部】cost_function_L — 被 line_search_newton_1d() 大量
//   呼叫（1D Newton 線搜索每一步都要評估 3 次：f(a)、f(a+δ)、f(a-δ)）
//   Lagrangian 值本身；同時把 Dx_all 帶出，減少重複計算
//   L(V) = f(X) + w_d λᵀ (D − θ + S²)
//   同時回傳 Dx_all (避免重複 FK)
// =====================================================================
double CgSolver::cost_function_L(const Eigen::VectorXd& V, Eigen::VectorXd& Dx_all_out) const
{
  const double f = cost_function_F(V);
  const Eigen::VectorXd lambda = V.segment(num_X_, num_C_);
  const Eigen::VectorXd S      = V.segment(num_X_ + num_C_, num_C_);
  Dx_all_out = compute_Dx_all(V.head(num_X_));
  const Eigen::VectorXd h_vec =
      (Dx_all_out.array() - danger_threshold_ + S.array().square()).matrix();
  const double penalty = lambda.dot(h_vec) * wd_;
  return f + penalty;
}

// =====================================================================
// 【呼叫順序 3-2a-1 / 4-內部】cost_function_F — 薄包裝，被
//   compute_G_smooth()（算梯度用）和 cost_function_L()（算 Lagrangian
//   用）呼叫；本身只是呼叫下面的 cost_function_F_split() 再丟掉 fa/fb
//   f = pw·fA + (1−pw)·fB
// =====================================================================
double CgSolver::cost_function_F(const Eigen::VectorXd& V) const
{
  double f, fa, fb;
  cost_function_F_split(V, f, fa, fb);
  return f;
}

// 【呼叫順序 3-2a-2】cost_function_F_split — 真正算平滑成本的地方，
//   對 A 臂、B 臂各呼叫一次 cost_Xm()，用 path_weight_ 加權合併
// [NEW] 拆出 fa/fb 往外傳 (= MATLAB cost_function_F 內部值, 原僅 debug 印)
void CgSolver::cost_function_F_split(const Eigen::VectorXd& V,
                                     double& f, double& fa, double& fb) const
{
  // [MATLAB] X = reshape(V(1:num_X),12,[])'  → 第 m 列 = [Xa_m, Xb_m]
  Eigen::MatrixXd Xmat(M_, 12);
  for (int m = 0; m < M_; ++m)
    Xmat.row(m) = V.segment(idx_Xm(m), 12).transpose();
  const Eigen::MatrixXd Xa = Xmat.leftCols(6);
  const Eigen::MatrixXd Xb = Xmat.rightCols(6);

  fa = cost_Xm(Xa, oriPos_.leftCols(6),  X_H_.row(0), X_T_.row(0));
  fb = cost_Xm(Xb, oriPos_.rightCols(6), X_H_.row(1), X_T_.row(1));
  f  = path_weight_ * fa + (1.0 - path_weight_) * fb;
}

// =====================================================================
// 【呼叫順序 3-2a-3】cost_Xm — 被 cost_function_F_split() 呼叫兩次
//   (A 臂一次, B 臂一次)；是整條呼叫鏈的最底層，純算術不再往下呼叫
//   任何東西，也不涉及碰撞/FK — 只管軌跡「平不平滑」
//   單臂平滑成本 (position prior + 頭尾 boundary + neighbor)
// =====================================================================
double CgSolver::cost_Xm(const Eigen::MatrixXd& Xa, const Eigen::MatrixXd& Xori,
                         const Eigen::RowVectorXd& XH, const Eigen::RowVectorXd& XT) const
{
  double c = 0.0;
  // (1) Position prior
  for (int m = 0; m < M_; ++m) {
    const Eigen::RowVectorXd dv = Xa.row(m) - Xori.row(m);
    c += smooth_w_ * dv.dot(dv);
  }
  // (2) Boundary(頭) + (3) Neighbor
  for (int m = 0; m < M_; ++m) {
    if (m == 0) {
      const Eigen::RowVectorXd dH = Xa.row(0) - XH;
      c += smooth_w_H_ * dH.dot(dH);
    } else {
      const Eigen::RowVectorXd dv = Xa.row(m) - Xa.row(m - 1);
      c += smooth_w_neighbor_ * dv.dot(dv);
    }
  }
  // (2) Boundary(尾)
  const Eigen::RowVectorXd dT = Xa.row(M_ - 1) - XT;
  c += smooth_w_T_ * dT.dot(dT);
  return c;
}

// =====================================================================
// 【呼叫順序 4】line_search_newton_1d — run_lag() 主迴圈「每次迭代第四步」
//   呼叫；給定當前點 V 和搜索方向 d，沿著這條線一維地用牛頓法找
//   讓 φ(α)=L(V+α·d) 最小的步長 α（內部自己是一個小迴圈，最多 2000 步）
//   1D Newton 線搜索 φ(α)=L(V+α·d)
//   ⚠ LS_DELTA=0.01, MAX_INNER=2000 (≠ ALM 之 0.001/50)
// =====================================================================
double CgSolver::line_search_newton_1d(const Eigen::VectorXd& V, const Eigen::VectorXd& d,
                                       int& ls_inner, bool& ls_fallback) const
{
  const double LS_DELTA       = 0.01;
  const int    MAX_INNER      = 2000;
  const double H_MIN_ABS      = 1e-9;
  const double ANEW_TOL       = 1e-3;
  const double FALLBACK_ALPHA = 0.0001;
  const int    FAIL_CHECK_IT  = 10;

  double a = 0.0;
  ls_fallback = false;
  ls_inner    = MAX_INNER;
  Eigen::VectorXd dummy;

  for (int k = 1; k <= MAX_INNER; ++k) {
    const double f0 = cost_function_L(V + a              * d, dummy);
    const double fp = cost_function_L(V + (a + LS_DELTA) * d, dummy);
    const double fm = cost_function_L(V + (a - LS_DELTA) * d, dummy);

    const double g_phi = (fp - fm) / (2.0 * LS_DELTA);
    const double h_phi = (fp - 2.0 * f0 + fm) / (LS_DELTA * LS_DELTA);

    if (std::abs(h_phi) < H_MIN_ABS) { ls_fallback = true; ls_inner = k; return FALLBACK_ALPHA; }

    const double anew = a - g_phi / h_phi;
    if (std::abs(anew - a) < ANEW_TOL) { a = anew; ls_inner = k; break; }
    a = anew;
    if (k > FAIL_CHECK_IT && (std::isnan(a) || std::isinf(a))) {
      ls_fallback = true; ls_inner = k; return FALLBACK_ALPHA;
    }
  }
  return a;
}

// =====================================================================
// 【呼叫順序 3】run_lag — 整個 CgSolver 的主迴圈，由
//   avoidance_system.cpp 的 run_solver_global() 呼叫「一次」
//   每次迭代依序呼叫: compute_D_cache() → compute_G() →
//   (CG 方向, 迴圈內聯) → line_search_newton_1d() → 更新 Xn →
//   收斂判定；下面 Step 1~12 的編號對應迴圈內程式碼段落
//   主迴圈 (CG Fletcher-Reeves + 1D Newton 線搜索)
//   = MATLAB Dual_Arm_Lagrangian_Con_v2.run_newton (C++ 更名 run_lag)
// =====================================================================
SolverLog CgSolver::run_lag()
{
  SolverLog log;

  Eigen::VectorXd Xn     = Xm_initial_;
  Eigen::VectorXd pre_Xn = Xn;

  // CG (Fletcher-Reeves) 狀態: 前一步梯度範數 + 前一步方向
  double pre_G_norm = 0.0;
  Eigen::VectorXd pre_d = Eigen::VectorXd::Zero(total_dim_);
  // 初始狀態
  Eigen::VectorXd Dx_init;
  double opt_L = cost_function_L(Xn, Dx_init);
  double max_D_curr = Dx_init.maxCoeff();
  double max_D_pre  = max_D_curr;

  log.max_D_init     = max_D_curr;
  log.violation_init = static_cast<int>((Dx_init.array() > danger_threshold_).count());

  Eigen::VectorXd Dx = Dx_init;
  int it = 0;

  for (it = 1; it <= max_solver_iter_; ++it) {
    // Step 1: FK 擾動快取
    Eigen::MatrixXd D_base; std::vector<Eigen::MatrixXd> D_plus;
    compute_D_cache(Xn, D_base, D_plus);

    // Step 2: 梯度 G = ∇L  (KKT 殘差)
    Eigen::VectorXd G = compute_G(Xn, D_base, D_plus);
    const double G_norm = G.norm();

    // Step 3: 方向 (Conjugate Gradient, Fletcher-Reeves)
    //   it>1 且 ‖pre_G‖>1e-9 → d = -G + β·pre_d,  β = ‖G‖²/‖pre_G‖²
    //   否則 (首輪或 pre_G 過小) → d = -G (最陡下降)
    //   ⚠ Con_v2 的非下降 restart (d·G>0 → d=-G) 為註解狀態, 此處比照不啟用
    Eigen::VectorXd d;
    if (it > 1 && pre_G_norm > 1e-9) {
      const double beta = (G_norm * G_norm) / (pre_G_norm * pre_G_norm);
      d = -G + beta * pre_d;
    } else {
      d = -G;
    }
    const double d_norm = d.norm();

    // Step 4: 發散偵測 (⚠ Con 用 d_norm, 非 G_norm)
    if (!d.allFinite() || d_norm > 1e9) {
      std::cout << "\n  ★ 發散! iter=" << it << " ||G||=" << G_norm
                << " ||d||=" << d_norm << "\n";
      log.diverge_iter = it;
      log.G_norm_history.push_back(G_norm);
      log.d_norm_history.push_back(d_norm);
      break;
    }

    // Step 5: 1D Newton 線搜索
    int ls_inner = 0; bool ls_fb = false;
    const double a = line_search_newton_1d(Xn, d, ls_inner, ls_fb);

    // Step 6: 更新 V 並評估
    Xn = pre_Xn + a * d;
    opt_L = cost_function_L(Xn, Dx);
    max_D_curr = Dx.maxCoeff();

    // Step 7: 滾動歷史
    pre_Xn = Xn;
    pre_G_norm = G_norm;   // CG: 記錄本步梯度範數
    pre_d      = d;        // CG: 記錄本步方向

    // Step 8: 拆 V 分量
    const Eigen::VectorXd lam_part = Xn.segment(num_X_, num_C_);
    const Eigen::VectorXd S_part   = Xn.segment(num_X_ + num_C_, num_C_);

    // Step 9: KKT 殘差 + 成本拆解 (L/f/fa/fb 往外傳)
    const Eigen::VectorXd G_X_part = G.head(num_X_);
    const Eigen::VectorXd h_vec =
        (Dx.array() - danger_threshold_ + S_part.array().square()).matrix();
    const double penalty = lam_part.dot(h_vec) * wd_;
    double f_now, fa_now, fb_now;
    cost_function_F_split(Xn, f_now, fa_now, fb_now);   // f_now == opt_L − penalty

    const double r_stat = G_X_part.norm();
    const double r_prim = h_vec.norm();
    const double r_comp = (lam_part.array() * S_part.array()).matrix().norm();
    const double r_dual = lam_part.cwiseMin(0.0).norm();
    const double lam_neg= lam_part.cwiseMin(0.0).sum();
    const double kkt_now= std::sqrt(r_stat*r_stat + r_prim*r_prim
                                    + r_comp*r_comp + r_dual*r_dual);
    const double G_lam_norm = G.segment(num_X_, num_C_).norm();
    const double G_S_norm   = G.segment(num_X_ + num_C_, num_C_).norm();
    const int    active_cnt = static_cast<int>((lam_part.array() > 1e-6).count());
    const int    viol_cnt   = static_cast<int>((Dx.array() > danger_threshold_).count());

    // Step 10: 顯示
    std::cout << "  [" << it << "] L:" << opt_L << " (f:" << f_now
              << "+pen:" << penalty << ") | MaxD:" << max_D_curr
              << " | |G|=" << G_norm << " |d|=" << d_norm
              << " | KKT:" << kkt_now << " (X:" << r_stat << " h:" << r_prim
              << " λS:" << r_comp << " λ−:" << lam_neg << ")"
              << " | λ=[" << lam_part.maxCoeff() << "," << lam_part.minCoeff() << "]"
              << " S=[" << S_part.maxCoeff() << "," << S_part.minCoeff() << "]"
              << " α:" << a << "\n";

    // Step 11: 寫 log (恆寫純量歷史, 供 export)
    log.L_history.push_back(opt_L);
    log.f_history.push_back(f_now);
    log.fa_history.push_back(fa_now);
    log.fb_history.push_back(fb_now);
    log.penalty_history.push_back(penalty);
    log.maxD_history.push_back(max_D_curr);
    log.G_norm_history.push_back(G_norm);
    log.d_norm_history.push_back(d_norm);
    log.alpha_history.push_back(a);
    log.r_stat_history.push_back(r_stat);
    log.r_prim_history.push_back(r_prim);
    log.r_comp_history.push_back(r_comp);
    log.r_dual_history.push_back(r_dual);
    log.lam_neg_history.push_back(lam_neg);
    log.kkt_history.push_back(kkt_now);
    log.G_lam_norm_history.push_back(G_lam_norm);
    log.G_S_norm_history.push_back(G_S_norm);
    log.lam_max_history.push_back(lam_part.maxCoeff());
    log.lam_min_history.push_back(lam_part.minCoeff());
    log.S_max_history.push_back(S_part.maxCoeff());
    log.S_min_history.push_back(S_part.minCoeff());
    log.active_count_history.push_back(active_cnt);
    log.violation_count_history.push_back(viol_cnt);
    log.ls_inner_history.push_back(ls_inner);
    log.ls_fallback_history.push_back(ls_fb ? 1 : 0);

    // Step 12: 收斂判定 (phys_ok && stable_ok; stat_ok 刻意停用 — 鞍點不收斂)
    const bool phys_ok   = max_D_curr <= danger_threshold_ + TOL_PHYS_MARGIN_;
    const bool stable_ok = std::abs(max_D_curr - max_D_pre) <= TOL_STABLE_;
    if (phys_ok && stable_ok) {
      std::cout << "\n  ★ 收斂! iter=" << it << " max_D=" << max_D_curr
                << " |ΔmaxD|=" << std::abs(max_D_curr - max_D_pre) << "\n";
      log.converge_iter = it;
      break;
    }

    if (it == max_solver_iter_)
      std::cout << "\n  ⚠ 達最大迭代 " << it << ", max_D=" << max_D_curr << " (未收斂)\n";

    max_D_pre = max_D_curr;
  }

  // 收尾
  const int actual_iter = std::min(it, max_solver_iter_);
  log.total_iter = actual_iter;

  V_final_ = Xn;
  X_final_ = Xn.head(num_X_);
  log.final_D    = compute_Dx_all(X_final_);
  log.lam_final  = Xn.segment(num_X_, num_C_);
  log.S_final    = Xn.segment(num_X_ + num_C_, num_C_);
  log.max_D_final     = log.final_D.maxCoeff();
  log.violation_final = static_cast<int>((log.final_D.array() > danger_threshold_).count());

  std::cout << "\n========== Con v2 結果摘要 ==========\n"
            << "  總迭代: " << actual_iter << " / " << max_solver_iter_ << "\n";
  if (log.converge_iter > 0)
    std::cout << "  狀態: ✅ 收斂於 iter " << log.converge_iter << "\n";
  else if (log.diverge_iter > 0)
    std::cout << "  狀態: ❌ 發散於 iter " << log.diverge_iter << "\n";
  else
    std::cout << "  狀態: ⚠ 達上限未收斂\n";
  std::cout << "  max_D: 初 " << log.max_D_init << " → 末 " << log.max_D_final
            << " (θ=" << danger_threshold_ << ")\n"
            << "  Violations: 初 " << log.violation_init << " → 末 "
            << log.violation_final << " / " << num_C_ << "\n"
            << "===========================================\n\n";

  return log;
}

}  // namespace dual_arm_lag_cg_planner
