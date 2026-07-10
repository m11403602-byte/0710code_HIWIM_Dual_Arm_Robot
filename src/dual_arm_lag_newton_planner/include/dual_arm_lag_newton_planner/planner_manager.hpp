// =====================================================================
// planner_manager.hpp — 第 3 層: MoveIt2 PlannerManager/PlanningContext
// =====================================================================
//   MoveIt2 插件介面封裝 (薄包裝, 把 AvoidanceSystem 接到 MoveIt)
//   ⚠ 演算法內部全 degree; MoveIt 介面全 radian -> 在 solve() 邊界轉換
//   ⚠ 關節名: A 臂 big_joint_1~6, B 臂 small_joint_1~6 (依你的 SRDF 調整)
// =====================================================================
#ifndef DUAL_ARM_LAG_NEWTON_PLANNER_PLANNER_MANAGER_HPP
#define DUAL_ARM_LAG_NEWTON_PLANNER_PLANNER_MANAGER_HPP

#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <rclcpp/rclcpp.hpp>

#include "dual_arm_lag_newton_planner/avoidance_system.hpp"
#include <string>

namespace dual_arm_lag_newton_planner
{

// =====================================================================
// PlanningContext: 單次規劃請求的執行單元 (solve)
// =====================================================================
class DualArmPlanningContext : public planning_interface::PlanningContext
{
public:
  DualArmPlanningContext(const std::string& name, const std::string& group,
                         const rclcpp::Node::SharedPtr& node)
    : planning_interface::PlanningContext(name, group), node_(node) {}

  // 主要規劃進入點
  bool solve(planning_interface::MotionPlanResponse& res) override;
  // DetailedResponse 委託給上面 (舊版模式)
  bool solve(planning_interface::MotionPlanDetailedResponse& res) override;

  bool terminate() override { return true; }   // 本規劃器不支援中途停止
  void clear() override {}

  // 從 yaml 傳入的參數
  double path_weight_         = 0.5;
  double danger_threshold_    = 0.4;
  double collision_tolerance_ = 0.1;
  double fix_tolerance_       = 0.1;
  int    max_refinement_iter_ = 15;
  double smooth_w_            = 0.3;
  double smooth_w_H_          = 1.0;
  double smooth_w_T_          = 1.0;
  double smooth_w_neighbor_   = 1.0;
  std::string joint_prefix_A_ = "big_joint_";
  std::string joint_prefix_B_ = "small_joint_";
  // 時間參數化
  bool   time_optimal_      = true;
  double path_total_time_   = 20.0;
  double min_time_interval_ = 0.05;

  // [NEW] 診斷輸出 (yaml 開關, 預設全關)
  std::string export_csv_prefix_;       // 非空 → 每次規劃後匯出 CSV
  bool        solver_verbose_ = false;  // 透傳內層 solver (記錄 G/d 全向量)
  int         export_level_   = 0;      // [NEW] 0=不匯出, 1=標配 6 檔, 2=完整 9 檔
  // [NEW] 純 Lagrangian 參數 (預設 = Gradient_v2 值)
  double lag_wd_ = 1.0; double lag_lam0_ = 30.0; double lag_s0_ = 1.0;
  double lag_tol_phys_ = 0.01; double lag_tol_stable_ = 0.01; double lag_tol_stat_ = 0.1; int lag_max_iter_ = 500;

private:
  rclcpp::Node::SharedPtr node_;
};

// =====================================================================
// PlannerManager: 插件入口 (MoveIt 載入這個)
// =====================================================================
class DualArmLagNewtonPlannerManager : public planning_interface::PlannerManager
{
public:
  DualArmLagNewtonPlannerManager() = default;
  ~DualArmLagNewtonPlannerManager() override = default;

  // ⚠ 參數順序: (model, node, ns) — 舊版已驗證
  bool initialize(const moveit::core::RobotModelConstPtr& model,
                  const rclcpp::Node::SharedPtr& node,
                  const std::string& parameter_namespace) override;

  std::string getDescription() const override { return "Dual-Arm Avoidance Planner (Lagrangian-Newton)"; }

  void getPlanningAlgorithms(std::vector<std::string>& algs) const override;

  void setPlannerConfigurations(const planning_interface::PlannerConfigurationMap& pcs) override;

  planning_interface::PlanningContextPtr getPlanningContext(
      const planning_scene::PlanningSceneConstPtr& planning_scene,
      const planning_interface::MotionPlanRequest& req,
      moveit_msgs::msg::MoveItErrorCodes& error_code) const override;

  bool canServiceRequest(const planning_interface::MotionPlanRequest& req) const override;

private:
  // 從參數伺服器重讀所有參數 (initialize 與每次 getPlanningContext 都呼叫,
  //   讓 yaml/rqt 改了下次規劃即生效)。const 因 getPlanningContext 是 const。
  void load_parameters() const;

  rclcpp::Node::SharedPtr node_;
  moveit::core::RobotModelConstPtr robot_model_;
  std::string parameter_namespace_;
  // mutable: load_parameters() 為 const 但需更新這些值
  mutable double path_weight_         = 0.5;
  mutable double danger_threshold_    = 0.4;
  mutable double collision_tolerance_ = 0.1;
  mutable double fix_tolerance_       = 0.1;
  mutable int    max_refinement_iter_ = 15;
  mutable double smooth_w_            = 0.3;
  mutable double smooth_w_H_          = 1.0;
  mutable double smooth_w_T_          = 1.0;
  mutable double smooth_w_neighbor_   = 1.0;
  mutable std::string joint_prefix_A_ = "big_joint_";
  mutable std::string joint_prefix_B_ = "small_joint_";
  mutable bool   time_optimal_      = true;
  mutable double path_total_time_   = 20.0;
  mutable double min_time_interval_ = 0.05;
  mutable std::string export_csv_prefix_;       // [NEW] 非空 → 規劃後匯出 CSV
  mutable bool        solver_verbose_ = false;  // [NEW] 內層 verbose log
  mutable int         export_level_   = 0;      // [NEW] 匯出等級
  mutable double lag_wd_ = 1.0; mutable double lag_lam0_ = 30.0; mutable double lag_s0_ = 1.0;
  mutable double lag_tol_phys_ = 0.01; mutable double lag_tol_stable_ = 0.01; mutable double lag_tol_stat_ = 0.1; mutable int lag_max_iter_ = 500;
};

}  // namespace dual_arm_lag_newton_planner

#endif
