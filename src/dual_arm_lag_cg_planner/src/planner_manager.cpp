// =====================================================================
// planner_manager.cpp — MoveIt2 插件實作 (= 整個插件的「大門」)
// =====================================================================
//   本檔是 MoveIt2 唯一會直接呼叫的地方；下面 avoidance_system.cpp /
//   cg_solver.cpp 都是被本檔間接呼叫的「內部引擎」，不會被 MoveIt 碰到。
//
//   ★ 完整呼叫順序（一次規劃請求，由上而下）★
//   ---------------------------------------------------------------
//   [A] move_group 啟動時，只執行一次：
//     A1. DualArmLagCgPlannerManager::initialize()      (本檔 L~167)
//           └─ load_parameters()                         (本檔 L~186)
//
//   [B] 使用者在 RViz 按下「Plan」，每次規劃請求都重跑一次：
//     B1. DualArmLagCgPlannerManager::getPlanningContext() (本檔 L~221)
//           ├─ load_parameters()                          (重讀 yaml)
//           └─ 建立 DualArmPlanningContext, 把參數搬進去
//     B2. MoveIt 呼叫 context->solve(res)
//           = DualArmPlanningContext::solve()              (本檔 L~18)
//           ├─ 1) 取出 start/goal RobotState
//           ├─ 2) radian → degree, 組出 A_wp/B_wp (2x6)
//           ├─ 3) 建立 AvoidanceSystem (見 avoidance_system.cpp 開頭註解)
//           ├─ 4) optimizer.set_lag_params() + run_optimization()
//           │      ← 這一步做完所有避障最佳化, 是最耗時的部分
//           ├─ 5) (可選) export_unified() 匯出 CSV
//           ├─ 6) degree → radian, 組回 RobotTrajectory
//           └─ 7) 時間參數化 (TOTG 或等間隔)，回傳給 MoveIt
//   ---------------------------------------------------------------
//   若想追完整演算法，讀完本檔後接著看 avoidance_system.cpp 開頭的
//   呼叫順序表，再看 cg_solver.cpp 開頭的呼叫順序表。
//   完整圖解版流程 + 數學公式，見同資料夾的 CODE_WALKTHROUGH.md。
// =====================================================================
#include "dual_arm_lag_cg_planner/planner_manager.hpp"
#include <exception>   // [REVISE] 匯出 try/catch

#include <pluginlib/class_list_macros.hpp>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <chrono>
#include <cmath>

namespace dual_arm_lag_cg_planner
{

// =====================================================================
// 【呼叫順序 B2】PlanningContext::solve — 主規劃流程
//   由 MoveIt2 的 move_group 節點直接呼叫 (使用者按 RViz「Plan」時觸發)
//   內部依序做 1~7 步 (見檔案最上方 B2 小節)，第 4 步呼叫 AvoidanceSystem
//   是整個規劃器真正在算避障軌跡的地方
// =====================================================================
bool DualArmPlanningContext::solve(planning_interface::MotionPlanResponse& res)
{
  // 1. 計時
  auto start_time = std::chrono::steady_clock::now();

  // 2. 檢查 goal constraints 有效性
  if (request_.goal_constraints.empty() ||
      request_.goal_constraints[0].joint_constraints.empty()) {
    res.error_code_.val = moveit_msgs::msg::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS;
    return false;
  }

  // 3. 從 MoveIt 提取起點/終點
  moveit::core::RobotState start_state = planning_scene_->getCurrentState();
  moveit::core::robotStateMsgToRobotState(request_.start_state, start_state);
  moveit::core::RobotState goal_state(start_state);
  for (const auto& jc : request_.goal_constraints[0].joint_constraints)
    goal_state.setJointPositions(jc.joint_name, &jc.position);
  goal_state.update();

  // 4. 轉成 Eigen 矩陣 (⚠ MoveIt 用 radian, 演算法用 degree)
  //    關節名前綴由 yaml 參數 joint_prefix_A/B 決定 (依 SRDF)
  Eigen::MatrixXd A_wp(2, 6), B_wp(2, 6);
  for (int j = 0; j < 6; ++j) {
    const std::string jA = joint_prefix_A_ + std::to_string(j + 1);
    const std::string jB = joint_prefix_B_ + std::to_string(j + 1);
    A_wp(0, j) = start_state.getJointPositions(jA)[0] * 180.0 / M_PI;
    A_wp(1, j) = goal_state.getJointPositions(jA)[0]  * 180.0 / M_PI;
    B_wp(0, j) = start_state.getJointPositions(jB)[0] * 180.0 / M_PI;
    B_wp(1, j) = goal_state.getJointPositions(jB)[0]  * 180.0 / M_PI;
  }

  // 5. 呼叫避障系統 (核心庫) — 傳入全部 yaml 可調參數
  AvoidanceSystem optimizer(A_wp, B_wp, path_weight_, danger_threshold_,
                            collision_tolerance_, fix_tolerance_, max_refinement_iter_,
                            smooth_w_, smooth_w_H_, smooth_w_T_, smooth_w_neighbor_);
  optimizer.set_lag_params(lag_wd_, lag_lam0_, lag_s0_,
                           lag_tol_phys_, lag_tol_stable_, lag_max_iter_);   // [NEW] yaml → 純 Lagrangian 注入
  optimizer.run_optimization();

  // [NEW] CSV 匯出工具 (export_csv_prefix 非空才動作)
  //   ⚠ 刻意安排在「純路徑規劃時間」計時區之外呼叫 — 磁碟 I/O 不污染對比數據
  auto export_csv_if_enabled = [&]() {
    if (export_csv_prefix_.empty()) { return; }
    // [REVISE] 整合匯出包 try/catch: 匯出是診斷副作用, 失敗(如權限/磁碟)只警告, 絕不擊落規劃
    try {
      optimizer.export_unified(export_csv_prefix_, export_level_);
    } catch (const std::exception& e) {
      RCLCPP_WARN(node_->get_logger(),
                  "[CSV] 匯出失敗, 規劃不受影響 (檢查 export_csv_prefix 是否可寫): %s", e.what());
    }
  };

  if (optimizer.has_collision()) {
    export_csv_if_enabled();   // [NEW] 失敗也匯出 (除錯素材; 失敗路徑無計時語意)
    res.error_code_.val = moveit_msgs::msg::MoveItErrorCodes::PLANNING_FAILED;
    return false;
  }

  // 6. 結果轉回 RobotTrajectory (⚠ degree -> radian)
  //    先暫填等間隔時間 (稍後依 time_optimal 重新參數化)
  auto trajectory = std::make_shared<robot_trajectory::RobotTrajectory>(
      start_state.getRobotModel(), getGroupName());
  const Trajectory& opt = optimizer.get_optimized_trajectory();
  for (int i = 0; i < opt.pos.rows(); ++i) {
    moveit::core::RobotState wp = start_state;
    for (int j = 0; j < 6; ++j) {
      double rad_A = opt.posA(i, j) * M_PI / 180.0;
      double rad_B = opt.posB(i, j) * M_PI / 180.0;
      const std::string jA = joint_prefix_A_ + std::to_string(j + 1);
      const std::string jB = joint_prefix_B_ + std::to_string(j + 1);
      wp.setJointPositions(jA, &rad_A);
      wp.setJointPositions(jB, &rad_B);
    }
    wp.update();
    trajectory->addSuffixWayPoint(wp, 0.1);   // 暫填, 下面會重設
  }

  // 7. 記錄純路徑規劃時間 (只含避障 run_optimization + 軌跡轉換, 不含時間參數化)
  auto plan_end = std::chrono::steady_clock::now();
  const double pure_plan_time = std::chrono::duration<double>(plan_end - start_time).count();
  std::cout << "\n[純路徑規劃時間] " << pure_plan_time << " 秒 (僅避障)\n";

  export_csv_if_enabled();   // [NEW] 計時結束後才寫盤 — 規劃時間數據保持乾淨

  // 8. 時間參數化 (在插件內處理; yaml 已移除 TOTG adapter, 避免重複)
  const int n_wp = static_cast<int>(trajectory->getWayPointCount());
  if (time_optimal_) {
    // ===== TOTG: 時間最佳化參數化 (依速度/加速度限制算時間戳) =====
    //   讀 RViz / MotionPlanRequest 的 scaling (滑桿), 預設/異常時用 1.0
    double vel_scale = request_.max_velocity_scaling_factor;
    double acc_scale = request_.max_acceleration_scaling_factor;
    if (vel_scale <= 0.0 || vel_scale > 1.0) vel_scale = 1.0;
    if (acc_scale <= 0.0 || acc_scale > 1.0) acc_scale = 1.0;

    trajectory_processing::TimeOptimalTrajectoryGeneration totg;
    bool ok = totg.computeTimeStamps(*trajectory, vel_scale, acc_scale);
    if (!ok) {
      std::cout << "  [WARN] TOTG 時間參數化失敗, 改用等間隔\n";
      for (int i = 1; i < n_wp; ++i)
        trajectory->setWayPointDurationFromPrevious(i, min_time_interval_);
    } else {
      std::cout << "  [時間參數化] TOTG (time_optimal=true, vel_scale="
                << vel_scale << ", acc_scale=" << acc_scale << ")\n";
    }
  } else {
    // ===== 自訂等間隔: dt = path_total_time/(n-1), 但不小於 min_time_interval =====
    double dt = (n_wp > 1) ? (path_total_time_ / (n_wp - 1)) : min_time_interval_;
    if (dt < min_time_interval_) dt = min_time_interval_;   // 受最小間隔保護
    for (int i = 1; i < n_wp; ++i)
      trajectory->setWayPointDurationFromPrevious(i, dt);
    std::cout << "  [時間參數化] 等間隔 dt=" << dt << " 秒 (time_optimal=false, "
              << "目標總時間=" << path_total_time_ << ", 最小間隔=" << min_time_interval_ << ")\n";
  }

  // 9. 顯示軌跡時長 (時間參數化後的真實執行總時長, getDuration)
  const double traj_duration = trajectory->getDuration();
  std::cout << "[軌跡時長] " << traj_duration << " 秒 (" << n_wp << " 點, 機器人執行用)\n";

  // 10. 軌跡規劃時長 = 純路徑規劃 + 時間參數化計算耗時 (電腦計算總時間)
  auto end_time = std::chrono::steady_clock::now();
  const double total_plan_time = std::chrono::duration<double>(end_time - start_time).count();
  std::cout << "[軌跡規劃時長] " << total_plan_time
            << " 秒 (純規劃 " << pure_plan_time
            << " + 時間參數化 " << (total_plan_time - pure_plan_time) << ")\n";

  res.trajectory_ = trajectory;
  res.error_code_.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
  res.planning_time_ = total_plan_time;   // 回報含時間參數化的總規劃時間
  return true;
}

// 【呼叫順序 B2'】DetailedResponse 委託 (舊版 MoveIt API 相容用)
//   有些舊版 MoveIt 呼叫端用這個簽名；內部直接轉呼叫上面的 solve()，
//   不是另一套邏輯
bool DualArmPlanningContext::solve(planning_interface::MotionPlanDetailedResponse& res)
{
  planning_interface::MotionPlanResponse normal_res;
  bool success = solve(normal_res);
  if (success) {
    res.trajectory_.push_back(normal_res.trajectory_);
    res.description_.push_back("DualArmAvoidancePlanner");
    res.processing_time_.push_back(normal_res.planning_time_);
  }
  res.error_code_ = normal_res.error_code_;
  return success;
}

// =====================================================================
// PlannerManager
// =====================================================================
// 【呼叫順序 A1】initialize — move_group 啟動、載入這個插件時呼叫「一次」
//   之後每次規劃並不會再呼叫這個函式；常駐參數的初值在這裡讀一次
bool DualArmLagCgPlannerManager::initialize(const moveit::core::RobotModelConstPtr& model,
                                       const rclcpp::Node::SharedPtr& node,
                                       const std::string& parameter_namespace)
{
  robot_model_         = model;
  node_                = node;
  parameter_namespace_ = parameter_namespace;

  load_parameters();   // 啟動時讀一次 (之後每次 getPlanningContext 會再重讀)

  RCLCPP_INFO(node_->get_logger(),
      "DualArmAvoidancePlanner initialized (path_weight=%.2f, danger_threshold=%.2f, "
      "collision_tol=%.2f, fix_tol=%.2f, max_iter=%d, smooth_w=%.2f, jointA='%s', jointB='%s')",
      path_weight_, danger_threshold_, collision_tolerance_, fix_tolerance_,
      max_refinement_iter_, smooth_w_, joint_prefix_A_.c_str(), joint_prefix_B_.c_str());
  return true;
}

// 【呼叫順序 A1a / B1a】load_parameters — 從參數伺服器重讀所有參數
//   會被呼叫兩種時機: (1) initialize() 時一次 (2) 之後每次
//   getPlanningContext() 都重呼叫一次，因此 yaml/rqt 改了參數，
//   下次按 Plan 就會生效，不用重啟 move_group
void DualArmLagCgPlannerManager::load_parameters() const
{
  const std::string ns = parameter_namespace_.empty() ? "" : (parameter_namespace_ + ".");
  node_->get_parameter_or(ns + "path_weight",         path_weight_,         0.5);
  node_->get_parameter_or(ns + "danger_threshold",    danger_threshold_,    0.35);
  node_->get_parameter_or(ns + "collision_tolerance", collision_tolerance_, 0.15);
  node_->get_parameter_or(ns + "fix_tolerance",       fix_tolerance_,       0.1);
  node_->get_parameter_or(ns + "max_refinement_iter", max_refinement_iter_, 15);
  node_->get_parameter_or(ns + "smooth_w",            smooth_w_,            0.3);
  node_->get_parameter_or(ns + "smooth_w_H",          smooth_w_H_,          1.0);
  node_->get_parameter_or(ns + "smooth_w_T",          smooth_w_T_,          1.0);
  node_->get_parameter_or(ns + "smooth_w_neighbor",   smooth_w_neighbor_,   1.0);
  node_->get_parameter_or(ns + "joint_prefix_A",      joint_prefix_A_,      std::string("big_joint_"));
  node_->get_parameter_or(ns + "joint_prefix_B",      joint_prefix_B_,      std::string("small_joint_"));
  node_->get_parameter_or(ns + "time_optimal",        time_optimal_,        true);
  node_->get_parameter_or(ns + "path_total_time",     path_total_time_,     20.0);
  node_->get_parameter_or(ns + "min_time_interval",   min_time_interval_,   0.05);
  node_->get_parameter_or(ns + "export_csv_prefix",   export_csv_prefix_,   std::string("./lag_data"));   // [NEW]
  node_->get_parameter_or(ns + "export_level",        export_level_,        0);                 // [NEW]
  node_->get_parameter_or(ns + "lag_lam0",            lag_lam0_,            30.0);
  node_->get_parameter_or(ns + "lag_max_iter",        lag_max_iter_,        500);
}

// 【呼叫順序】getPlanningAlgorithms — MoveIt 查詢「這個插件支援哪些演算法名稱」
//   用於 RViz Planning Library 下拉選單顯示（本規劃器固定只回報一種）
void DualArmLagCgPlannerManager::getPlanningAlgorithms(std::vector<std::string>& algs) const
{
  algs.clear();
  algs.push_back("DualArmAvoidanceLagCG");
}

// 【呼叫順序】setPlannerConfigurations — MoveIt 介面要求實作的空掛鉤
//   本規劃器沒有「多套 per-config 參數」概念 (參數只從 yaml 讀一份)，故留空
void DualArmLagCgPlannerManager::setPlannerConfigurations(
    const planning_interface::PlannerConfigurationMap& /*pcs*/)
{
  // 本規劃器無額外 per-config 設定; 參數已在 initialize 讀入
}

// 【呼叫順序 B1】getPlanningContext — 每次規劃請求的入口
//   RViz 按下「Plan」時 MoveIt 先呼叫這裡「準備」一個 Context，
//   準備好之後 MoveIt 才會呼叫 context->solve()（見上方 solve() 的
//   【呼叫順序 B2】註解）。這裡做的事：
//     1) load_parameters() 重讀 yaml（讓每次規劃都用最新參數）
//     2) new 一個 DualArmPlanningContext，把讀到的參數逐一搬進去
planning_interface::PlanningContextPtr DualArmLagCgPlannerManager::getPlanningContext(
    const planning_scene::PlanningSceneConstPtr& planning_scene,
    const planning_interface::MotionPlanRequest& req,
    moveit_msgs::msg::MoveItErrorCodes& error_code) const
{
  if (!planning_scene) {
    RCLCPP_ERROR(node_->get_logger(), "planning_scene 為空");
    error_code.val = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
    return planning_interface::PlanningContextPtr();
  }

  // 每次規劃前重讀參數 (讓 yaml/rqt 動態調整生效, 不用重啟 move_group)
  load_parameters();

  auto context = std::make_shared<DualArmPlanningContext>(
      "dual_arm_avoidance_context", req.group_name, node_);
  context->setPlanningScene(planning_scene);
  context->setMotionPlanRequest(req);
  context->path_weight_         = path_weight_;
  context->danger_threshold_    = danger_threshold_;
  context->collision_tolerance_ = collision_tolerance_;
  context->fix_tolerance_       = fix_tolerance_;
  context->max_refinement_iter_ = max_refinement_iter_;
  context->smooth_w_            = smooth_w_;
  context->smooth_w_H_          = smooth_w_H_;
  context->smooth_w_T_          = smooth_w_T_;
  context->smooth_w_neighbor_   = smooth_w_neighbor_;
  context->joint_prefix_A_      = joint_prefix_A_;
  context->joint_prefix_B_      = joint_prefix_B_;
  context->time_optimal_        = time_optimal_;
  context->path_total_time_     = path_total_time_;
  context->min_time_interval_   = min_time_interval_;
  context->export_csv_prefix_   = export_csv_prefix_;   // [NEW]
  context->export_level_        = export_level_;        // [NEW]
  context->lag_wd_   = lag_wd_;   context->lag_lam0_ = lag_lam0_;     // [NEW] 純 Lagrangian 六參數
  context->lag_s0_   = lag_s0_;
  context->lag_tol_phys_ = lag_tol_phys_; context->lag_tol_stable_ = lag_tol_stable_;
  context->lag_max_iter_ = lag_max_iter_;

  error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
  return context;
}

// 【呼叫順序】canServiceRequest — MoveIt 在選規劃器前先問「你能接這個請求嗎」
//   本規劃器只認「有關節目標」的請求 (joint goal constraints)，
//   Cartesian/姿態目標一律回 false，MoveIt 會改用其他規劃器
bool DualArmLagCgPlannerManager::canServiceRequest(
    const planning_interface::MotionPlanRequest& req) const
{
  // 只服務有 joint goal constraints 的請求
  return !req.goal_constraints.empty() &&
         !req.goal_constraints[0].joint_constraints.empty();
}

}  // namespace dual_arm_lag_cg_planner

// =====================================================================
// pluginlib 註冊 (MoveIt 透過這個找到插件)
// =====================================================================
PLUGINLIB_EXPORT_CLASS(
    dual_arm_lag_cg_planner::DualArmLagCgPlannerManager,
    planning_interface::PlannerManager)
