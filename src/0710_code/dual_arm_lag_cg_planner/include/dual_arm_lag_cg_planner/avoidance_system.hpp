// =====================================================================
// avoidance_system.hpp — 第 2 層: 外層碰撞修復系統 (= MATLAB System v3)
// =====================================================================
//   外層碰撞修復迴圈: Clamped Spline 初始軌跡 -> 碰撞偵測 -> 找 targets ->
//   呼叫內層 CG 優化 -> Spline 重建 -> 重新檢查 (最多 max_refinement_iter 輪)
//   含 CSV export (除畫圖外的所有資料匯出)
//
//   [MATLAB] 對應 class: Dual_Arm_avoidance_system_v3
//   全 degree; 不依賴 MoveIt (純數學, 可獨立使用)
// =====================================================================
#ifndef DUAL_ARM_LAG_CG_PLANNER_AVOIDANCE_SYSTEM_HPP
#define DUAL_ARM_LAG_CG_PLANNER_AVOIDANCE_SYSTEM_HPP

#include "dual_arm_lag_cg_planner/cg_solver.hpp"
#include <Eigen/Dense>
#include <vector>
#include <string>

namespace dual_arm_lag_cg_planner
{

// [MATLAB] trajectory struct
struct Trajectory {
  Eigen::VectorXd time;   // (T)   步數索引
  Eigen::MatrixXd posA;   // (Tx6) A 臂關節角
  Eigen::MatrixXd posB;   // (Tx6) B 臂關節角
  Eigen::MatrixXd pos;    // (Tx12) [posA, posB]
};

// [MATLAB] find_collision_targets 回傳的 indices
struct CollisionIndices {
  int minidx = 0;
  int maxidx = 0;
  std::vector<int> targets;   // 5 個控制點 (0-indexed): [Head, q1, peak, q3, Tail]
};

// [MATLAB] iter_log entry (每輪外層修復的快照, 給 export 用)
struct IterLogEntry {
  Trajectory          traj_in;       // 修復前軌跡
  Trajectory          traj_out;      // 修復後軌跡
  Eigen::VectorXd     path_D_max_in;  // 修復前每步 max_D
  Eigen::VectorXd     path_D_max_out; // 修復後每步 max_D
  std::vector<int>    targets_in;     // 5 點 (修復前 index)
  std::vector<int>    targets_out;    // 5 點 (新軌跡 index)
  SolverLog           solver_log;     // 該輪內層 CG log
};

class AvoidanceSystem
{
public:
  // [MATLAB] 建構函數 (A_waypoints, B_waypoints, path_weight, DANGER_THRESHOLD=0.4)
  //   A_waypoints/B_waypoints: 2x6 (起點列 + 終點列), degree
  AvoidanceSystem(const Eigen::MatrixXd& A_waypoints,
                  const Eigen::MatrixXd& B_waypoints,
                  double path_weight,
                  double danger_threshold    = 0.35,
                  // 以下為可調參數 (帶預設值, 不傳則用 MATLAB 預設)
                  double collision_tolerance = 0.1,
                  double fix_tolerance       = 0.1,
                  int    max_refinement_iter = 15,
                  double smooth_w            = 0.3,
                  double smooth_w_H          = 1.0,
                  double smooth_w_T          = 1.0,
                  double smooth_w_neighbor   = 1.0);

  // [MATLAB] run_optimization: 碰撞修復主迴圈
  void run_optimization();

  // ===== Getter (對外只暴露 getter) =====
  const Trajectory& get_optimized_trajectory() const { return trajectory_opt_; }
  const Trajectory& get_original_trajectory()  const { return trajectory_ori_; }
  bool has_collision() const { return is_collision_; }


  // [NEW] 純 Lagrangian 參數注入 (yaml 可調; 不呼叫則用 solver 預設)
  //   ⚠ λ/S 是決策變數初值 (非 ALM 的外層乘子); 與 ALM 之 set_alm_params 不同概念
  void set_lag_params(double wd, double lam0, double s0,
                      double tol_phys_margin, double tol_stable, int max_iter)
  {
    lag_wd_ = wd; lag_lam0_ = lam0; lag_s0_ = s0;
    lag_tol_phys_ = tol_phys_margin; lag_tol_stable_ = tol_stable;
    lag_max_iter_ = max_iter;
  }
  bool is_optimized()  const { return is_optimized_; }

  // ===== CSV Export ([REVISE] 整合為單一入口, 舊四匯出器已刪除) =====
  // [NEW] export_unified: 整合匯出 (對外唯一入口)
  //   level 1 = 論文標配 6 檔, level 2 = +constraints_all/path_original/path_evolution
  void export_unified(const std::string& prefix, int level) const;

private:
  // ===== 機器人/問題設定 =====
  Eigen::Matrix4d robotA_base_ = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d robotB_base_ = Eigen::Matrix4d::Identity();
  Eigen::MatrixXd A_waypoints_;   // 2x6
  Eigen::MatrixXd B_waypoints_;   // 2x6

  double danger_threshold_    = 0.35;
  double path_weight_         = 0.5;
  double collision_tolerance_ = 0.15;
  double fix_tolerance_       = 0.1;
  double STEP_MAX_DEG_        = 0.5;
  int    max_refinement_iter_ = 15;
  // 平滑權重 (轉傳給 CgSolver)
  double smooth_w_            = 0.3;
  double smooth_w_H_          = 1.0;
  double smooth_w_T_          = 1.0;
  double smooth_w_neighbor_   = 1.0;

  // ===== 狀態 =====
  Trajectory      trajectory_ori_;
  Trajectory      trajectory_opt_;
  Eigen::VectorXd path_D_ori_;       // (T) 原始每步 max_D
  Eigen::VectorXd path_D_opt_;
  Eigen::MatrixXd path_D_all_ori_;   // (T x num_D) 原始全約束 D (export 用)
  bool   is_collision_ = true;
  bool   is_optimized_ = false;
  int    refinement_count_ = 0;
  // [NEW] 純 Lagrangian 參數 (預設 = Gradient_v2 值)
  double lag_wd_         = 1.0;     // 對偶強度 (penalty 係數)
  double lag_lam0_       = 30.0;    // λ_0 (⚠ 註解寫 10, 實際 code = 30)
  double lag_s0_         = 1.0;     // S_0 (S²=1)
  double lag_tol_phys_   = 0.01;    // max_D ≤ θ + margin
  double lag_tol_stable_ = 0.01;   // |Δmax_D| ≤ TOL_STABLE
  int    lag_max_iter_   = 500;
  std::vector<double> refinement_history_;
  std::vector<double> time_ms_;            // 每輪內層耗時
  std::vector<IterLogEntry> iter_log_;

  // ===== 私有方法 (= MATLAB private) =====
  void generate_initial_trajectory();
  void check_collision(const Trajectory& traj,
                       Eigen::VectorXd& path_D_max, bool& is_collision,
                       Eigen::MatrixXd* path_D_all = nullptr) const;
  CollisionIndices find_collision_targets(const Eigen::VectorXd& path_D) const;
  Trajectory regenerate_trajectory_global(const Trajectory& old_traj,
                                           const Eigen::MatrixXd& Xa_opt,
                                           const Eigen::MatrixXd& Xb_opt,
                                           const CollisionIndices& indices,
                                           std::vector<int>& targets_out) const;
  // 呼叫內層 CG: 回傳 Xa_opt/Xb_opt (M x 6) + log
  void run_solver_global(const Trajectory& traj, const std::vector<int>& targets,
                         Eigen::MatrixXd& Xa_opt, Eigen::MatrixXd& Xb_opt,
                         SolverLog& solver_log);

  // ===== Clamped Cubic Spline 工具 =====
  // 給 t_knots (n), 值 Y (dim x n), 端點斜率 v0/v1 (dim), 查詢點 t_query (q)
  //   -> 回傳 (q x dim) 插值結果
  static Eigen::MatrixXd clamped_cubic_spline(const Eigen::VectorXd& t_knots,
                                              const Eigen::MatrixXd& Y,
                                              const Eigen::VectorXd& v0,
                                              const Eigen::VectorXd& v1,
                                              const Eigen::VectorXd& t_query);
};

}  // namespace dual_arm_lag_cg_planner

#endif
