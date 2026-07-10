// =====================================================================
// planner_manager.cpp — MoveIt2 插件實作
// =====================================================================
#include "dual_arm_alm_gd_planner/planner_manager.hpp"
#include <exception>   // [REVISE] 匯出 try/catch

#include <pluginlib/class_list_macros.hpp>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <chrono>
#include <cmath>

namespace dual_arm_alm_gd_planner
{

// =====================================================================
// PlanningContext::solve — 主規劃流程
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
  optimizer.set_alm_params(alm_mu0_, alm_c0_, alm_c_max_,
                           alm_beta_c_, alm_gamma_v_);   // [NEW] yaml → ALM 參數注入
  optimizer.set_alm_iters(alm_k_outer_, alm_k_inner_);   // [NEW] yaml → 迭代上限注入
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

// DetailedResponse 委託 (舊版模式)
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
bool DualArmAlmGdPlannerManager::initialize(const moveit::core::RobotModelConstPtr& model,
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

// 從參數伺服器重讀所有參數 (yaml/rqt 改了下次規劃即生效)
void DualArmAlmGdPlannerManager::load_parameters() const
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
  node_->get_parameter_or(ns + "export_csv_prefix",   export_csv_prefix_,   std::string("./alm_data"));   // [NEW]
  node_->get_parameter_or(ns + "export_level",        export_level_,        0);                 // [NEW]
  node_->get_parameter_or(ns + "alm_mu0",             alm_mu0_,             10.0);              // [NEW] ALM 六參數
  node_->get_parameter_or(ns + "alm_c0",              alm_c0_,              5.0);
  node_->get_parameter_or(ns + "alm_c_max",           alm_c_max_,           2000.0);
  node_->get_parameter_or(ns + "alm_beta_c",          alm_beta_c_,          2.0);
  node_->get_parameter_or(ns + "alm_gamma_v",         alm_gamma_v_,         0.5);
  node_->get_parameter_or(ns + "alm_k_outer",         alm_k_outer_,         50);
  node_->get_parameter_or(ns + "alm_k_inner",         alm_k_inner_,         200);
}

void DualArmAlmGdPlannerManager::getPlanningAlgorithms(std::vector<std::string>& algs) const
{
  algs.clear();
  algs.push_back("DualArmAvoidanceGD");
}

void DualArmAlmGdPlannerManager::setPlannerConfigurations(
    const planning_interface::PlannerConfigurationMap& /*pcs*/)
{
  // 本規劃器無額外 per-config 設定; 參數已在 initialize 讀入
}

planning_interface::PlanningContextPtr DualArmAlmGdPlannerManager::getPlanningContext(
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
  context->alm_mu0_  = alm_mu0_;  context->alm_c0_     = alm_c0_;      // [NEW] ALM 六參數
  context->alm_c_max_ = alm_c_max_;
  context->alm_beta_c_ = alm_beta_c_; context->alm_gamma_v_ = alm_gamma_v_;
  context->alm_k_outer_ = alm_k_outer_; context->alm_k_inner_ = alm_k_inner_;

  error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
  return context;
}

bool DualArmAlmGdPlannerManager::canServiceRequest(
    const planning_interface::MotionPlanRequest& req) const
{
  // 只服務有 joint goal constraints 的請求
  return !req.goal_constraints.empty() &&
         !req.goal_constraints[0].joint_constraints.empty();
}

}  // namespace dual_arm_alm_gd_planner

// =====================================================================
// pluginlib 註冊 (MoveIt 透過這個找到插件)
// =====================================================================
PLUGINLIB_EXPORT_CLASS(
    dual_arm_alm_gd_planner::DualArmAlmGdPlannerManager,
    planning_interface::PlannerManager)
