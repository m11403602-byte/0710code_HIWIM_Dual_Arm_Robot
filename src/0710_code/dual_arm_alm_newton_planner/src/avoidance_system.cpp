// =====================================================================
// avoidance_system.cpp — 第 2 層外層系統實作 (= MATLAB System v3)
// =====================================================================
//   ⚠ MATLAB 1-indexed -> C++ 0-indexed: targets/索引全程 0-indexed,
//     僅在對應 MATLAB 公式時註明
// =====================================================================
#include "dual_arm_alm_newton_planner/avoidance_system.hpp"
#include "dual_arm_alm_newton_planner/data_io.hpp"
#include <filesystem>   // [NEW] export_unified 子目錄
#include <ctime>        // [NEW] 目錄名時間戳

#include <cmath>
#include <iostream>
#include <algorithm>
#include <limits>
#include <chrono>

namespace dual_arm_alm_newton_planner
{

// =====================================================================
// Clamped Cubic Spline (取代 MATLAB spline 的 clamped 模式)
// =====================================================================
// [MATLAB] spline(t, [v0, Y, v1], tq): 給端點斜率 -> clamped cubic
//   標準解法: 解三對角系統求各 knot 二階導 M, 再分段 Hermite 求值
//   每個維度獨立解 (Y 是 dim x n)
Eigen::MatrixXd AvoidanceSystem::clamped_cubic_spline(const Eigen::VectorXd& t_knots,
                                                      const Eigen::MatrixXd& Y,
                                                      const Eigen::VectorXd& v0,
                                                      const Eigen::VectorXd& v1,
                                                      const Eigen::VectorXd& t_query)
{
  const int n   = static_cast<int>(t_knots.size());   // knot 數
  const int dim = static_cast<int>(Y.rows());          // 維度 (12)
  const int q   = static_cast<int>(t_query.size());
  Eigen::MatrixXd out(q, dim);

  // 區間長度 h_i = t[i+1]-t[i]
  Eigen::VectorXd h(n - 1);
  for (int i = 0; i < n - 1; ++i) h(i) = t_knots(i+1) - t_knots(i);

  for (int dd = 0; dd < dim; ++dd) {
    // y 值
    Eigen::VectorXd y(n);
    for (int i = 0; i < n; ++i) y(i) = Y(dd, i);

    // 解二階導 M (clamped: 已知 y'(0)=v0, y'(end)=v1)
    // 三對角系統 A*M = rhs (n x n)
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(n, n);
    Eigen::VectorXd rhs = Eigen::VectorXd::Zero(n);

    if (n == 2) {
      // 單段: 直接用兩端點 + 兩端斜率 (Hermite), M 解 2x2
      // clamped 條件:
      //   2*h0*M0 + h0*M1 = 6*((y1-y0)/h0 - v0)
      //   h0*M0 + 2*h0*M1 = 6*(v1 - (y1-y0)/h0)
      const double h0 = h(0);
      A(0,0) = 2.0*h0; A(0,1) = h0;
      A(1,0) = h0;     A(1,1) = 2.0*h0;
      rhs(0) = 6.0 * ((y(1)-y(0))/h0 - v0(dd));
      rhs(1) = 6.0 * (v1(dd) - (y(1)-y(0))/h0);
    } else {
      // 內部節點: 標準二階導連續方程
      for (int i = 1; i < n - 1; ++i) {
        A(i, i-1) = h(i-1);
        A(i, i)   = 2.0 * (h(i-1) + h(i));
        A(i, i+1) = h(i);
        rhs(i) = 6.0 * ((y(i+1)-y(i))/h(i) - (y(i)-y(i-1))/h(i-1));
      }
      // clamped 邊界
      A(0,0) = 2.0*h(0); A(0,1) = h(0);
      rhs(0) = 6.0 * ((y(1)-y(0))/h(0) - v0(dd));
      A(n-1, n-2) = h(n-2); A(n-1, n-1) = 2.0*h(n-2);
      rhs(n-1) = 6.0 * (v1(dd) - (y(n-1)-y(n-2))/h(n-2));
    }

    Eigen::VectorXd M = A.colPivHouseholderQr().solve(rhs);

    // 分段求值
    for (int k = 0; k < q; ++k) {
      const double tq = t_query(k);
      // 找區間 i: t[i] <= tq <= t[i+1]
      int i = 0;
      while (i < n - 2 && tq > t_knots(i+1)) ++i;
      const double hi = h(i);
      const double a  = (t_knots(i+1) - tq) / hi;
      const double b  = (tq - t_knots(i)) / hi;
      // 三次樣條公式
      const double val = a*y(i) + b*y(i+1)
        + ((a*a*a - a)*M(i) + (b*b*b - b)*M(i+1)) * (hi*hi) / 6.0;
      out(k, dd) = val;
    }
  }
  return out;
}

// =====================================================================
// 建構函數 (= MATLAB System v3 建構函數)
// =====================================================================
AvoidanceSystem::AvoidanceSystem(const Eigen::MatrixXd& A_waypoints,
                                 const Eigen::MatrixXd& B_waypoints,
                                 double path_weight,
                                 double danger_threshold,
                                 double collision_tolerance,
                                 double fix_tolerance,
                                 int    max_refinement_iter,
                                 double smooth_w,
                                 double smooth_w_H,
                                 double smooth_w_T,
                                 double smooth_w_neighbor)
  : A_waypoints_(A_waypoints), B_waypoints_(B_waypoints),
    danger_threshold_(danger_threshold), path_weight_(path_weight),
    collision_tolerance_(collision_tolerance), fix_tolerance_(fix_tolerance),
    max_refinement_iter_(max_refinement_iter),
    smooth_w_(smooth_w), smooth_w_H_(smooth_w_H),
    smooth_w_T_(smooth_w_T), smooth_w_neighbor_(smooth_w_neighbor)
{
  // [輸出即時化] 關閉 std::cout 緩衝, 確保 [Init]/[Outer] 等訊息在
  //   MoveIt (非終端機環境) 下也立即印出, 不被緩衝累積 (避免「有時不印」)
  std::cout << std::unitbuf;

  // [MATLAB] base: A 臂 Ty(700)Rz(180), B 臂 Ty(-700)Rz(0)
  robotA_base_ = NewtonSolver::make_translation('y', 700) * NewtonSolver::make_rotation('z', 180);
  robotB_base_ = NewtonSolver::make_translation('y', -700) * NewtonSolver::make_rotation('z', 0);

  // STEP_MAX_DEG_ 維持預設 (本批未開放為參數)
  STEP_MAX_DEG_ = 0.5;

  generate_initial_trajectory();
}

// =====================================================================
// generate_initial_trajectory: Clamped Cubic Spline (2 waypoints)
// =====================================================================
void AvoidanceSystem::generate_initial_trajectory()
{
  const Eigen::MatrixXd& wA = A_waypoints_;
  const Eigen::MatrixXd& wB = B_waypoints_;

  // [MATLAB] Anchors = [wA, wB] (2 x 12)
  Eigen::MatrixXd Anchors(2, 12);
  Anchors.leftCols(6)  = wA;
  Anchors.rightCols(6) = wB;

  // [MATLAB] dist = max(norm(diff(wA)), norm(diff(wB)))
  double distA = (wA.row(1) - wA.row(0)).norm();
  double distB = (wB.row(1) - wB.row(0)).norm();
  double dist  = std::max(distA, distB);
  if (dist < 1e-4) dist = 1e-6;

  // [MATLAB] n_steps = max(1, ceil(dist/STEP_MAX_DEG)); T = n_steps+1
  const int n_steps = std::max(1, static_cast<int>(std::ceil(dist / STEP_MAX_DEG_)));
  const int T_total = n_steps + 1;

  Eigen::VectorXd t_knots(2); t_knots << 0.0, dist;
  Eigen::VectorXd t_query = Eigen::VectorXd::LinSpaced(T_total, 0.0, dist);

  // [MATLAB] clamped: v_start=v_end=0; Y = Anchors' (12 x 2)
  Eigen::VectorXd v0 = Eigen::VectorXd::Zero(12);
  Eigen::VectorXd v1 = Eigen::VectorXd::Zero(12);
  Eigen::MatrixXd Y = Anchors.transpose();   // 12 x 2

  Eigen::MatrixXd Pos = clamped_cubic_spline(t_knots, Y, v0, v1, t_query);  // T x 12

  trajectory_ori_.time = Eigen::VectorXd::LinSpaced(T_total, 0, T_total - 1);
  trajectory_ori_.posA = Pos.leftCols(6);
  trajectory_ori_.posB = Pos.rightCols(6);
  trajectory_ori_.pos  = Pos;

  std::cout << "  [Init] 初始軌跡: 弦長 " << dist << "deg, 切分 " << T_total << " 點\n";
}

// =====================================================================
// check_collision (B 方案: 動態探 D 長度)
// =====================================================================
void AvoidanceSystem::check_collision(const Trajectory& traj,
                                      Eigen::VectorXd& path_D_max, bool& is_collision,
                                      Eigen::MatrixXd* path_D_all) const
{
  const int T = static_cast<int>(traj.time.size());

  // [MATLAB] 先算第 1 步取 D 長度 (用 calc_df_bubble_ver1 mode=3 全約束)
  //   C++ 用 NewtonSolver 的 FK+calc_df+mask 走一次, 取得 num_D
  auto compute_step_D = [&](int t) -> Eigen::VectorXd {
    double Ja[6], Jb[6];
    for (int j = 0; j < 6; ++j) { Ja[j] = traj.posA(t, j); Jb[j] = traj.posB(t, j); }
    Eigen::MatrixXd bA, bB; Eigen::VectorXd rA, rB; Eigen::Matrix4d eeA, eeB;
    NewtonSolver::robot_arm_bubble_RA610(robotA_base_, Ja, bA, rA, eeA);
    NewtonSolver::robot_arm_bubble_RA605(robotB_base_, Jb, bB, rB, eeB);
    Eigen::MatrixXd sj = NewtonSolver::calc_df(rA, rB, bA, bB);   // 16x18
    auto mask = NewtonSolver::get_collision_masks();
    // column-major 取 mask 內元素 (與 newton_solver 一致)
    std::vector<double> vals;
    for (int col = 0; col < 18; ++col)
      for (int row = 0; row < 16; ++row)
        if (mask(row, col)) vals.push_back(sj(row, col));
    Eigen::VectorXd D(vals.size());
    for (size_t k = 0; k < vals.size(); ++k) D(k) = vals[k];
    return D;
  };

  Eigen::VectorXd D0 = compute_step_D(0);
  const int D_len = static_cast<int>(D0.size());

  Eigen::MatrixXd allD(T, D_len);
  allD.row(0) = D0.transpose();
  for (int t = 1; t < T; ++t) allD.row(t) = compute_step_D(t).transpose();

  path_D_max = allD.rowwise().maxCoeff();   // (T) 每步 max

  const double thr = danger_threshold_ + collision_tolerance_;   // 0.5
  is_collision = (path_D_max.array() >= thr).any();

  if (path_D_all) *path_D_all = allD;
}

// =====================================================================
// find_collision_targets: 找最危險段 + 5 個特徵點 (0-indexed)
// =====================================================================
CollisionIndices AvoidanceSystem::find_collision_targets(const Eigen::VectorXd& path_D) const
{
  const double threshold = danger_threshold_;   // [MATLAB] 用 0.4 (非 +tol)
  const int n = static_cast<int>(path_D.size());

  // [MATLAB] all_danger_indices = find(path_D >= threshold) (0-indexed)
  std::vector<int> danger;
  for (int i = 0; i < n; ++i)
    if (path_D(i) >= threshold) danger.push_back(i);

  std::vector<int> index;   // 最危險段的索引集
  if (danger.empty()) {
    // [MATLAB] 全低於閾值 -> 取 max 單點
    int mx = 0;
    for (int i = 1; i < n; ++i) if (path_D(i) > path_D(mx)) mx = i;
    index.push_back(mx);
  } else {
    // [MATLAB] 用 diff>1 拆連續段, 挑最危險段
    std::vector<int> seg_starts, seg_ends;
    seg_starts.push_back(0);
    for (size_t k = 0; k + 1 < danger.size(); ++k)
      if (danger[k+1] - danger[k] > 1) { seg_ends.push_back(static_cast<int>(k)); seg_starts.push_back(static_cast<int>(k+1)); }
    seg_ends.push_back(static_cast<int>(danger.size()) - 1);

    double max_seen = -std::numeric_limits<double>::infinity();
    std::vector<int> best_seg;
    for (size_t s = 0; s < seg_starts.size(); ++s) {
      std::vector<int> seg(danger.begin()+seg_starts[s], danger.begin()+seg_ends[s]+1);
      double cur_max = -std::numeric_limits<double>::infinity();
      for (int idx : seg) cur_max = std::max(cur_max, path_D(idx));
      if (cur_max > max_seen) { max_seen = cur_max; best_seg = seg; }
    }
    index = best_seg;
  }

  int minidx = *std::min_element(index.begin(), index.end());
  int maxidx = *std::max_element(index.begin(), index.end());

  // peak (Max_idx)
  double Max_val = -std::numeric_limits<double>::infinity();
  int Max_idx = index[0];
  for (int idx : index) if (path_D(idx) > Max_val) { Max_val = path_D(idx); Max_idx = idx; }

  // [MATLAB] q1: minidx..Max_idx-1 找 D 接近 (threshold+Max_val)/2
  double mean_err = std::numeric_limits<double>::infinity();
  int q1_idx = (minidx + Max_idx) / 2;
  for (int it = minidx; it <= Max_idx - 1; ++it) {
    double val = std::abs(path_D(it) - (threshold + Max_val) / 2.0);
    if (val < mean_err) { q1_idx = it; mean_err = val; }
  }
  // [MATLAB] q3: maxidx..Max_idx+1 反向掃
  mean_err = std::numeric_limits<double>::infinity();
  int q3_idx = (Max_idx + maxidx) / 2;
  for (int it = maxidx; it >= Max_idx + 1; --it) {
    double val = std::abs(path_D(it) - (threshold + Max_val) / 2.0);
    if (val < mean_err) { q3_idx = it; mean_err = val; }
  }

  // [MATLAB] 邊界保護 minidx=max(_,2)/maxidx=min(_,len-1) (1-indexed)
  //   0-indexed: minidx>=1, maxidx<=n-2
  minidx = std::max(minidx, 1);
  maxidx = std::min(maxidx, n - 2);

  // [MATLAB] fix_gap = max(round(fix_tol*(maxidx-minidx)), 1)
  int fix_gap = std::max(static_cast<int>(std::lround(fix_tolerance_ * (maxidx - minidx))), 1);
  int Head = minidx - fix_gap;
  int Tail = maxidx + fix_gap;
  Head = std::max(Head, 0);       // [MATLAB] max(Head,1) -> 0-indexed max(.,0)
  Tail = std::min(Tail, n - 1);   // [MATLAB] min(Tail,len) -> 0-indexed min(.,n-1)

  q1_idx = std::max(minidx, std::min(q1_idx, Max_idx));
  q3_idx = std::max(Max_idx, std::min(q3_idx, maxidx));

  CollisionIndices ci;
  ci.minidx = minidx;
  ci.maxidx = maxidx;
  ci.targets = {Head, q1_idx, Max_idx, q3_idx, Tail};

  std::cout << "  [Targets] t1=" << Head << "(D=" << path_D(Head) << ") q1=" << q1_idx
            << " peak=" << Max_idx << "(D=" << Max_val << ") q3=" << q3_idx
            << " t5=" << Tail << " gap=" << fix_gap << "\n";
  return ci;
}

// =====================================================================
// run_solver_global: 呼叫內層 Newton
// =====================================================================
void AvoidanceSystem::run_solver_global(const Trajectory& traj, const std::vector<int>& targets,
                                        Eigen::MatrixXd& Xa_opt, Eigen::MatrixXd& Xb_opt,
                                        SolverLog& solver_log)
{
  auto t1 = std::chrono::steady_clock::now();

  // [MATLAB] X = trajectory.pos(targets, :) (5 x 12)
  Eigen::MatrixXd X(static_cast<int>(targets.size()), 12);
  for (size_t i = 0; i < targets.size(); ++i) X.row(static_cast<int>(i)) = traj.pos.row(targets[i]);

  NewtonSolver solver(X, robotA_base_, robotB_base_, danger_threshold_, path_weight_,
                  smooth_w_, smooth_w_H_, smooth_w_T_, smooth_w_neighbor_);
  solver.set_alm_params(alm_mu0_, alm_c0_, alm_c_max_,
                        alm_beta_c_, alm_gamma_v_);   // [NEW] yaml → ALM 注入 (乘子更新為純 PHR, 無 mu 上界/reset)
  solver.set_iter_limits(alm_k_outer_, alm_k_inner_);   // [NEW] yaml → 迭代上限注入
  solver_log = solver.run_alm();

  // [MATLAB] P = size(X,1)-2; Xm = reshape(V_final(1:P*12),12,[])' (column-major)
  const int P = static_cast<int>(targets.size()) - 2;
  const Eigen::VectorXd& V = solver.get_X_final();
  Xa_opt.resize(P, 6);
  Xb_opt.resize(P, 6);
  for (int m = 0; m < P; ++m) {
    for (int j = 0; j < 6; ++j) {
      Xa_opt(m, j) = V(m * 12 + j);
      Xb_opt(m, j) = V(m * 12 + 6 + j);
    }
  }

  auto t2 = std::chrono::steady_clock::now();
  time_ms_.push_back(std::chrono::duration<double, std::milli>(t2 - t1).count());
}

// =====================================================================
// regenerate_trajectory_global: 局部 Spline 重建 + C1 邊界斜率對齊
// =====================================================================
Trajectory AvoidanceSystem::regenerate_trajectory_global(const Trajectory& old_traj,
                                                         const Eigen::MatrixXd& Xa_opt,
                                                         const Eigen::MatrixXd& Xb_opt,
                                                         const CollisionIndices& indices,
                                                         std::vector<int>& targets_out) const
{
  const std::vector<int>& targets = indices.targets;
  const Eigen::MatrixXd& pos_all = old_traj.pos;
  const int total_len = static_cast<int>(pos_all.rows());
  const int P = static_cast<int>(Xa_opt.rows());

  // [MATLAB] Patch_Anchors = [Head點; 優化點(P); Tail點] ((P+2) x 12)
  Eigen::MatrixXd Patch(P + 2, 12);
  Patch.row(0) = pos_all.row(targets.front());
  for (int m = 0; m < P; ++m) {
    Patch.block(m+1, 0, 1, 6) = Xa_opt.row(m);
    Patch.block(m+1, 6, 1, 6) = Xb_opt.row(m);
  }
  Patch.row(P + 1) = pos_all.row(targets.back());

  // [MATLAB] 段距離 + valid_mask>1e-4 過濾重合
  //   valid_indices = [true; valid_mask], sum(valid_indices) = 保留的 anchor 數
  std::vector<int> valid_rows; valid_rows.push_back(0);
  std::vector<double> seg_dists;
  for (int i = 1; i < P + 2; ++i) {
    double d = (Patch.row(i) - Patch.row(i-1)).norm();
    if (d > 1e-4) { valid_rows.push_back(i); seg_dists.push_back(d); }
  }

  // [MATLAB] if sum(valid_indices) < 2: 退化 (anchor 幾乎全重合)
  //   Clean_Anchors = [Patch(1,:); Patch(end,:)] (只留頭尾)
  //   Clean_dist_seg = 1e-6;  t_knots = [0; 1e-6]
  Eigen::MatrixXd Clean;
  Eigen::VectorXd t_knots;
  if (static_cast<int>(valid_rows.size()) < 2) {
    Clean.resize(2, 12);
    Clean.row(0) = Patch.row(0);          // 頭 (Patch_Anchors(1,:))
    Clean.row(1) = Patch.row(P + 1);      // 尾 (Patch_Anchors(end,:))
    seg_dists.assign(1, 1e-6);            // Clean_dist_seg = 1e-6
    t_knots.resize(2);
    t_knots << 0.0, 1e-6;                 // [0; 1e-6]
  } else {
    // [MATLAB] else: Clean = Patch(valid_indices,:); t_knots = [0; cumsum(dist_seg)]
    Clean.resize(static_cast<int>(valid_rows.size()), 12);
    for (size_t i = 0; i < valid_rows.size(); ++i)
      Clean.row(static_cast<int>(i)) = Patch.row(valid_rows[i]);
    t_knots.resize(static_cast<int>(seg_dists.size()) + 1);
    t_knots(0) = 0.0;
    for (size_t i = 0; i < seg_dists.size(); ++i) t_knots(i+1) = t_knots(i) + seg_dists[i];
  }

  // [MATLAB] n_steps_all = ceil(seg/STEP_MAX); 去尾拼接
  std::vector<int> n_steps_all;
  for (double d : seg_dists) n_steps_all.push_back(std::max(1, static_cast<int>(std::ceil(d / STEP_MAX_DEG_))));
  int T_patch = 1; for (int s : n_steps_all) T_patch += s;

  Eigen::VectorXd t_query(T_patch);
  int cur = 0;
  for (size_t i = 0; i < n_steps_all.size(); ++i) {
    int len = n_steps_all[i];
    Eigen::VectorXd seg_t = Eigen::VectorXd::LinSpaced(len + 1, t_knots(i), t_knots(i+1));
    if (i < n_steps_all.size() - 1) {
      for (int k = 0; k < len; ++k) t_query(cur + k) = seg_t(k);   // 去尾
      cur += len;
    } else {
      for (int k = 0; k <= len; ++k) t_query(cur + k) = seg_t(k);  // 末段保尾
    }
  }

  // [MATLAB] C1 邊界斜率對齊 (接縫斜率, 防 overshoot)
  Eigen::VectorXd v_start = Eigen::VectorXd::Zero(12);
  Eigen::VectorXd v_end   = Eigen::VectorXd::Zero(12);
  if (targets.front() > 0)
    v_start = (pos_all.row(targets.front()) - pos_all.row(targets.front()-1)).transpose();
  if (targets.back() < total_len - 1)
    v_end = (pos_all.row(targets.back()+1) - pos_all.row(targets.back())).transpose();

  Eigen::MatrixXd Y = Clean.transpose();   // 12 x n_knots
  Eigen::MatrixXd Patch_Pos = clamped_cubic_spline(t_knots, Y, v_start, v_end, t_query);  // T_patch x 12

  // ===== 拼接 (完全對齊 MATLAB regenerate_trajectory_global) =====
  // [MATLAB] patch_core = Patch_Pos(2:end-1, :)  -> 去頭尾, 用「實際行數」
  //   num_patch = size(patch_core, 1) = Patch_Pos.rows() - 2  (★關鍵: 用實際行數, 非 T_patch)
  const int patch_rows = static_cast<int>(Patch_Pos.rows());
  const int num_patch  = patch_rows - 2;
  // [MATLAB] num_head = targets(1)  (1-indexed 位置值)
  //   C++ targets[0] 是 0-indexed, 故 MATLAB num_head = targets[0] + 1
  const int num_head = targets.front() + 1;
  // [MATLAB] num_tail = total_len - targets(end) + 1
  //   C++: total_len - (targets.back()+1) + 1 = total_len - targets.back()
  const int num_tail = total_len - targets.back();
  // [MATLAB] T_new = num_head + num_patch + num_tail  (無額外 +1)
  const int T_new = num_head + num_patch + num_tail;

  // [MATLAB] 整段賦值 (Eigen 區塊操作, 對應 MATLAB final_pos(a:b,:)=...)
  //   段3 已補 MATLAB 退化分支 (sum(valid)<2 -> 2點spline), 故此處 num_patch>=0
  Eigen::MatrixXd final_pos(T_new, 12);
  // final_pos(1:num_head, :) = pos_all(1:num_head, :)
  //   MATLAB 前 num_head 列 = C++ pos_all[0 .. num_head-1]
  final_pos.topRows(num_head) = pos_all.topRows(num_head);
  // final_pos(num_head+1 : num_head+num_patch, :) = patch_core (Patch_Pos 去頭尾)
  if (num_patch > 0)
    final_pos.middleRows(num_head, num_patch) = Patch_Pos.middleRows(1, num_patch);
  // final_pos(num_head+num_patch+1 : end, :) = pos_all(targets(end) : end, :)
  //   MATLAB targets(end) (1-indexed) = C++ targets.back() (0-indexed) 起算, 共 num_tail 列
  final_pos.bottomRows(num_tail) = pos_all.bottomRows(num_tail);

  Trajectory nt;
  nt.pos  = final_pos;
  nt.time = Eigen::VectorXd::LinSpaced(T_new, 0, T_new - 1);
  nt.posA = final_pos.leftCols(6);
  nt.posB = final_pos.rightCols(6);

  // [MATLAB] targets_out: 5 anchor 在新軌跡 index
  //   if all(valid_indices) && length(n_steps_all)==4
  targets_out.clear();
  if (static_cast<int>(valid_rows.size()) == P + 2 && n_steps_all.size() == 4) {
    // [MATLAB] cum_steps = cumsum(n_steps_all);
    //   targets_out = [num_head, num_head+cum(1), num_head+cum(2), num_head+cum(3), num_head+num_patch+1]
    //   ⚠ MATLAB num_head 是 1-indexed -> 轉 0-indexed 要 -1
    int c1 = n_steps_all[0], c2 = c1 + n_steps_all[1], c3 = c2 + n_steps_all[2];
    const int nh = num_head - 1;   // 轉回 0-indexed 基準
    targets_out = {nh, nh + c1, nh + c2, nh + c3, nh + num_patch + 1};
  } else {
    // [MATLAB] 退化: round(linspace(num_head, num_head+num_patch+1, 5))
    std::cout << "  [Regen] anchor 重合, targets_out 用近似\n";
    const int nh = num_head - 1;
    Eigen::VectorXd lin = Eigen::VectorXd::LinSpaced(5, nh, nh + num_patch + 1);
    for (int k = 0; k < 5; ++k) targets_out.push_back(static_cast<int>(std::lround(lin(k))));
  }

  std::cout << "  [Regenerate] 有效點: " << valid_rows.size()
            << ", 總點數: " << total_len << " -> " << T_new << "\n";
  return nt;
}

// =====================================================================
// run_optimization: 碰撞修復主迴圈 (含起終點前置檢查)
// =====================================================================
void AvoidanceSystem::run_optimization()
{
  check_collision(trajectory_ori_, path_D_ori_, is_collision_, &path_D_all_ori_);

  // [MATLAB] 起終點前置檢查 (0.4 閾值 + warning+return)
  if (path_D_ori_(0) >= danger_threshold_ + collision_tolerance_) {
    std::cout << "  [WARN] 起點姿態接近碰撞 (D=" << path_D_ori_(0) << " >= "
              << danger_threshold_ + collision_tolerance_<< "), 無法規劃\n";
    trajectory_opt_ = trajectory_ori_; path_D_opt_ = path_D_ori_;
    is_optimized_ = false; is_collision_ = true; return;
  }
  if (path_D_ori_(path_D_ori_.size()-1) >= danger_threshold_ + collision_tolerance_) {
    std::cout << "  [WARN] 終點姿態接近碰撞 (D=" << path_D_ori_(path_D_ori_.size()-1) << " >= "
              << danger_threshold_ + collision_tolerance_<< "), 目標不可達\n";
    trajectory_opt_ = trajectory_ori_; path_D_opt_ = path_D_ori_;
    is_optimized_ = false; is_collision_ = true; return;
  }

  if (!is_collision_) {
    std::cout << "  初始軌跡安全 (Max D=" << path_D_ori_.maxCoeff() << "), 無需優化\n";
    trajectory_opt_ = trajectory_ori_; path_D_opt_ = path_D_ori_;
    is_optimized_ = false; is_collision_ = false; return;
  }

  std::cout << "  初始軌跡偵測到碰撞 (Max D=" << path_D_ori_.maxCoeff() << "), 開始修復\n";
  refinement_history_.push_back(path_D_ori_.maxCoeff());

  Trajectory current = trajectory_ori_;
  Eigen::VectorXd current_D = path_D_ori_;

  while (true) {
    ++refinement_count_;
    std::cout << "\n--- 外層修復迭代 " << refinement_count_ << " ---\n";

    CollisionIndices ci = find_collision_targets(current_D);

    IterLogEntry entry;
    entry.traj_in = current;
    entry.path_D_max_in = current_D;
    entry.targets_in = ci.targets;

    Eigen::MatrixXd Xa_opt, Xb_opt; SolverLog slog;
    run_solver_global(current, ci.targets, Xa_opt, Xb_opt, slog);
    entry.solver_log = slog;

    if (slog.diverge_iter > 0) {
      std::cout << "  [WARN] 第" << refinement_count_ << "次優化發散\n";
      trajectory_opt_ = current; path_D_opt_ = current_D;
      is_optimized_ = true; is_collision_ = true; return;
    }

    std::vector<int> targets_out;
    current = regenerate_trajectory_global(current, Xa_opt, Xb_opt, ci, targets_out);
    // std::cout<< "  [Regenerate] 完成\n";
    check_collision(current, current_D, is_collision_);
    // std::cout<< "  [Check Collision] 完成\n";
    refinement_history_.push_back(current_D.maxCoeff());
    // std::cout<< "  [Record] 完成\n";

    entry.traj_out = current;
    entry.path_D_max_out = current_D;
    entry.targets_out = targets_out;
    iter_log_.push_back(entry);

    std::cout << "  外層 " << refinement_count_ << " 完成: Max D=" << current_D.maxCoeff() << "\n";

    if (!is_collision_) { std::cout << "\n成功: 軌跡已安全\n"; break; }

    if (refinement_count_ >= max_refinement_iter_) {
      std::cout << "  [WARN] 已達最大修復次數, 仍存在碰撞\n";
      trajectory_opt_ = current; path_D_opt_ = current_D;
      is_optimized_ = true; is_collision_ = true; return;
    }
  }

  trajectory_opt_ = current; path_D_opt_ = current_D;
  is_optimized_ = true; is_collision_ = false;
  std::cout << "\n外層修復完成: " << refinement_count_ << " 輪, "
            << "原始 " << trajectory_ori_.time.size() << " 點 -> 優化 "
            << trajectory_opt_.time.size() << " 點\n";
}

// =====================================================================
// export_unified: 整合匯出 [NEW] (= 舊四匯出器去重重組, 對外唯一入口)
//   目錄: <prefix>/<unix秒>_<SOLVER>/  (每次規劃一個子目錄)
//   level 0: 完全不匯出 (總開關)
//   level 1 (論文標配 6 檔): meta / summary / inner / danger_final /
//                            danger_rounds / targets
//   level 2 (深掘 +3 檔):    constraints_all / path_original / path_evolution
//   長表設計: 「每輪一檔」改為「round 欄長表」, 檔案數固定不隨輪數膨脹
// =====================================================================
void AvoidanceSystem::export_unified(const std::string& prefix, int level) const
{
  if (level <= 0) { return; }   // [NEW] level 0 = 完全不記錄 (總開關, 不用清 prefix)

  const char* SOLVER_NAME = "Newton";
  const std::string dir = prefix + "/" +
      std::to_string(static_cast<long>(std::time(nullptr))) + "_" + SOLVER_NAME;
  std::filesystem::create_directories(dir);

  const int n_outer = static_cast<int>(iter_log_.size());
  const double NaN = std::nan("");

  // ---- 1. meta.csv: 參數快照 (key,value 直欄; solver 名在目錄名) ----
  {
    std::vector<std::string> keys = {
      "danger_threshold","collision_tolerance","fix_tolerance","max_refinement_iter",
      "path_weight","smooth_w","smooth_w_H","smooth_w_T","smooth_w_neighbor",
      "alm_mu0","alm_c0","alm_c_max","alm_beta_c","alm_gamma_v",
      "STEP_MAX_DEG","n_rounds","T_ori","num_D","export_level"};
    Eigen::MatrixXd V(static_cast<int>(keys.size()), 1);
    V << danger_threshold_, collision_tolerance_, fix_tolerance_,
         static_cast<double>(max_refinement_iter_),
         path_weight_, smooth_w_, smooth_w_H_, smooth_w_T_, smooth_w_neighbor_,
         alm_mu0_, alm_c0_, alm_c_max_, alm_beta_c_, alm_gamma_v_,
         STEP_MAX_DEG_, static_cast<double>(n_outer),
         static_cast<double>(trajectory_ori_.time.size()),
         static_cast<double>(path_D_all_ori_.cols()),
         static_cast<double>(level);
    write_csv_labeled(dir + "/meta.csv", {"key","value"}, keys, V);
  }

  // ---- 2. summary.csv: 每輪計分板 (合併舊 #2 八欄 + 舊 #4 獨有六欄 = 14 欄) ----
  {
    Eigen::MatrixXd S(n_outer, 14);
    for (int k = 0; k < n_outer; ++k) {
      const SolverLog& lg = iter_log_[k].solver_log;
      const int idx = std::max(1, lg.outer_iter) - 1;   // 最後圈 (0-indexed)
      S(k,0)  = k + 1;
      S(k,1)  = lg.total_inner;
      S(k,2)  = lg.outer_iter;
      S(k,3)  = lg.outer_break_iter;
      S(k,4)  = lg.diverge_iter;
      S(k,5)  = lg.c_final;
      S(k,6)  = lg.maxD_history.empty()    ? 0.0 : lg.maxD_history[idx];
      S(k,7)  = (k < static_cast<int>(time_ms_.size())) ? time_ms_[k] : 0.0;
      S(k,8)  = lg.L_history.empty()       ? 0.0 : lg.L_history[idx];
      S(k,9)  = lg.f_history_f.empty()     ? 0.0 : lg.f_history_f[idx];
      S(k,10) = lg.f_history_fa.empty()    ? 0.0 : lg.f_history_fa[idx];
      S(k,11) = lg.f_history_fb.empty()    ? 0.0 : lg.f_history_fb[idx];
      S(k,12) = lg.final_G_norm;
      S(k,13) = lg.d_last_history.empty()  ? 0.0 : lg.d_last_history[idx];
    }
    write_csv(dir + "/summary.csv",
      {"round","total_inner","outer_iter","conv_iter","diverge_iter","c_final",
       "maxD_final","time_ms","L","f","f_a","f_b","final_G_norm","final_d_norm"}, S);
  }

  // ---- 3. inner.csv: 內層逐步串接 (= 舊 diag_inner) ----
  {
    int total = 0;
    for (const auto& e : iter_log_) total += e.solver_log.total_inner;
    Eigen::MatrixXd I(total, 5);
    int row = 0;
    for (int k = 0; k < n_outer; ++k) {
      const SolverLog& lg = iter_log_[k].solver_log;
      for (int s = 0; s < lg.total_inner; ++s) {
        I(row,0) = k + 1;
        I(row,1) = s + 1;
        I(row,2) = (s < static_cast<int>(lg.G_norm_history.size())) ? lg.G_norm_history[s] : 0.0;
        I(row,3) = (s < static_cast<int>(lg.d_norm_history.size())) ? lg.d_norm_history[s] : 0.0;
        I(row,4) = (s < static_cast<int>(lg.alpha_history.size()))  ? lg.alpha_history[s]  : 0.0;
        ++row;
      }
    }
    write_csv(dir + "/inner.csv", {"round","inner_step","G_norm","d_norm","alpha"}, I);
  }

  // ---- 4. danger_final.csv: 初始 vs 最終 MaxD 對照 (= 舊 #1) ----
  {
    const int To = static_cast<int>(trajectory_ori_.time.size());
    const int Tp = static_cast<int>(trajectory_opt_.time.size());
    const int mx = std::max(To, Tp);
    Eigen::MatrixXd M = Eigen::MatrixXd::Constant(mx, 4, NaN);
    for (int i = 0; i < To; ++i) { M(i,0) = i; M(i,1) = path_D_ori_(i); }
    for (int i = 0; i < Tp; ++i) { M(i,2) = i; M(i,3) = path_D_opt_(i); }
    write_csv(dir + "/danger_final.csv",
      {"Step_Original","MaxD_Original","Step_Optimized","MaxD_Optimized"}, M);
  }

  // ---- 5. danger_rounds.csv: 長表 round|step|MaxD_in|MaxD_out (取代每輪一檔) ----
  {
    int rows = 0;
    for (const auto& e : iter_log_)
      rows += std::max(static_cast<int>(e.path_D_max_in.size()),
                       static_cast<int>(e.path_D_max_out.size()));
    Eigen::MatrixXd D = Eigen::MatrixXd::Constant(std::max(rows,1), 4, NaN);
    int r = 0;
    for (int k = 0; k < n_outer; ++k) {
      const IterLogEntry& e = iter_log_[k];
      const int Ti = static_cast<int>(e.path_D_max_in.size());
      const int To = static_cast<int>(e.path_D_max_out.size());
      for (int i = 0; i < std::max(Ti, To); ++i) {
        D(r,0) = k + 1;
        D(r,1) = i;
        if (i < Ti) D(r,2) = e.path_D_max_in(i);
        if (i < To) D(r,3) = e.path_D_max_out(i);
        ++r;
      }
    }
    write_csv(dir + "/danger_rounds.csv", {"round","step","MaxD_in","MaxD_out"}, D);
  }

  // ---- 6. targets.csv: 長表 5 特徵點 x 每輪 (索引 + D 值 + in/out 關節角) ----
  if (n_outer > 0) {
    std::vector<std::string> hdr = {"Pt","round","idx_in","D_in","idx_out","D_out"};
    const char* pre[4] = {"Ain","Bin","Aout","Bout"};
    for (int p = 0; p < 4; ++p)
      for (int j = 1; j <= 6; ++j) hdr.push_back(std::string(pre[p]) + std::to_string(j));
    const char* pt_name[5] = {"Head","q1","peak","q3","Tail"};
    std::vector<std::string> labels;
    Eigen::MatrixXd T(5 * n_outer, 29);
    int row = 0;
    for (int k = 0; k < n_outer; ++k) {
      const IterLogEntry& e = iter_log_[k];
      for (int p = 0; p < 5; ++p) {
        labels.push_back(pt_name[p]);
        const int ti = e.targets_in[p], to = e.targets_out[p];
        T(row,0) = k + 1;
        T(row,1) = ti; T(row,2) = e.path_D_max_in(ti);
        T(row,3) = to; T(row,4) = e.path_D_max_out(to);
        for (int c = 0; c < 12; ++c) {
          T(row, 5 + c)  = e.traj_in.pos(ti, c);
          T(row, 17 + c) = e.traj_out.pos(to, c);
        }
        ++row;
      }
    }
    write_csv_labeled(dir + "/targets.csv", hdr, labels, T);
  }

  // ===== level 2: 深掘三檔 =====
  if (level >= 2) {
    // ---- 7. constraints_all.csv: 全約束大頭 (只此一份) ----
    {
      const int Dn = static_cast<int>(path_D_all_ori_.cols());
      Eigen::MatrixXd A(path_D_all_ori_.rows(), Dn + 1);
      A.col(0) = trajectory_ori_.time;
      A.rightCols(Dn) = path_D_all_ori_;
      std::vector<std::string> hdr; hdr.push_back("Step");
      for (int j = 0; j < Dn; ++j) { char buf[16]; snprintf(buf, 16, "DF_%03d", j+1); hdr.push_back(buf); }
      write_csv(dir + "/constraints_all.csv", hdr, A);
    }
    // ---- 8. path_original.csv ----
    {
      const Trajectory& t = trajectory_ori_;
      Eigen::MatrixXd P(t.time.size(), 13);
      P.col(0) = t.time;
      P.block(0,1,t.time.size(),6) = t.posA;
      P.block(0,7,t.time.size(),6) = t.posB;
      write_csv(dir + "/path_original.csv",
        {"Step","A1","A2","A3","A4","A5","A6","B1","B2","B3","B4","B5","B6"}, P);
    }
    // ---- 9. path_evolution.csv: 長表 round|step|A1..B6 (取代每輪 Path_RoundK) ----
    {
      int rows = 0;
      for (const auto& e : iter_log_) rows += static_cast<int>(e.traj_out.time.size());
      Eigen::MatrixXd P(std::max(rows,1), 14);
      P.setZero();
      int r = 0;
      for (int k = 0; k < n_outer; ++k) {
        const Trajectory& t = iter_log_[k].traj_out;
        for (int i = 0; i < static_cast<int>(t.time.size()); ++i) {
          P(r,0) = k + 1;
          P(r,1) = t.time(i);
          for (int j = 0; j < 6; ++j) { P(r,2+j) = t.posA(i,j); P(r,8+j) = t.posB(i,j); }
          ++r;
        }
      }
      write_csv(dir + "/path_evolution.csv",
        {"round","Step","A1","A2","A3","A4","A5","A6","B1","B2","B3","B4","B5","B6"}, P);
    }
  }

  std::cout << "[export_unified] " << (level >= 2 ? 9 : 6) << " 檔匯出至 " << dir << "/\n";
}

}  // namespace dual_arm_alm_newton_planner
