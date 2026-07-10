// 下行: 端點層透明轉送（詳見 transparent_relay_architecture.md）
//
// 核心理念: 中間人「不替任何人作答」— 只搬運訊息 + 改 joint 名。
//   D10 假端點(3 service + 2 topic) ←原樣轉交→ D20/D30 真 JTC 端點
//   goal_id(UUID) 全程不換 → 無對應表; 無狀態機; 失敗語意 = 直連。
//
// 設計要點:
// (1) 延遲回覆(deferred response): D10 的 service callback 只「押單」
//     (存號碼牌 rmw_request_id_t)即返回, executor 不被佔用;
//     手臂的真答案到貨時(D20/D30 執行緒)憑號碼牌 send_response 補答。
//     ⚠ 本檔唯一 API 賭注: create_service 的「帶 service 控柄之延遲回覆」
//       callback 簽名 (service, request_header, request) — rclcpp_action
//       內部即用此機制懸置 get_result; 若編譯不過, 改用 2 參數變體
//       (request_header, request) + 成員存 service 控柄。
// (2) 閉包接力: 每張請求單的下文由自己的 lambda 攜帶 → 本程式零全域可變狀態。
// (3) 內容改寫僅兩處: send_goal 請求的 Goal 去前綴(4 欄位)、feedback 加前綴。
// (4) status topic 必須鏡像 transient_local QoS(訂閱與發布兩側都要)。
// (5) 快速失敗: 遠端 service 不在線 → 立即回 accepted=false / ERROR_REJECTED,
//     補償「轉送存在會遮蔽手臂離線」的效應。
//
// 已知限制: Result.error_string 裸關節名不翻譯; 未經實機驗證。
// 用法: ros2 run dual_arm_domain_bridge trajectory_downlink_endpoint_relay [10 11 12]

#include "dual_arm_domain_bridge/multi_context.hpp"

#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>

#include <cstdlib>
#include <memory>
#include <string>
#include <utility>

using dual_arm_domain_bridge::DomainNode;
using dual_arm_domain_bridge::make_domain_node;
using FJT = control_msgs::action::FollowJointTrajectory;

// 五端點的實作層型別（rclcpp_action 靠它們運作, 穩定存在）
using SendGoalSrv = FJT::Impl::SendGoalService;
using CancelSrv = FJT::Impl::CancelGoalService;
using GetResultSrv = FJT::Impl::GetResultService;
using FeedbackMsg = FJT::Impl::FeedbackMessage;
using StatusMsg = FJT::Impl::GoalStatusMessage;

namespace
{

constexpr char kArmAction[] = "/joint_trajectory_controller/follow_joint_trajectory";

std::string strip_prefix(const std::string & s, const std::string & prefix)
{
  return (s.rfind(prefix, 0) == 0) ? s.substr(prefix.size()) : s;
}

// 下行內容改寫: Goal 內所有關節名去前綴（4 處）
void strip_goal(FJT::Goal & g, const std::string & prefix)
{
  for (auto & n : g.trajectory.joint_names) {n = strip_prefix(n, prefix);}
  for (auto & t : g.path_tolerance) {t.name = strip_prefix(t.name, prefix);}
  for (auto & t : g.goal_tolerance) {t.name = strip_prefix(t.name, prefix);}
  for (auto & n : g.multi_dof_trajectory.joint_names) {n = strip_prefix(n, prefix);}
}

// 一支手臂的五端點轉送（零全域狀態: 全靠閉包接力）
class ArmEndpointRelay
{
public:
  ArmEndpointRelay(
    const rclcpp::Node::SharedPtr & host_node, const rclcpp::Node::SharedPtr & arm_node,
    const std::string & host_action, std::string prefix)
  : prefix_(std::move(prefix)), logger_(host_node->get_logger())
  {
    const std::string arm_action(kArmAction);

    // ---- D20/D30 側: 對真 JTC 的 3 個 service client ----
    cl_send_goal_ = arm_node->create_client<SendGoalSrv>(arm_action + "/_action/send_goal");
    cl_cancel_ = arm_node->create_client<CancelSrv>(arm_action + "/_action/cancel_goal");
    cl_get_result_ = arm_node->create_client<GetResultSrv>(arm_action + "/_action/get_result");

    // ---- D10 側: 假端點 service ×3（延遲回覆模式: callback 不帶 Response）----
    // ① send_goal — 唯一要改寫請求內容的端點
    sv_send_goal_ = host_node->create_service<SendGoalSrv>(
      host_action + "/_action/send_goal",
      [this](const std::shared_ptr<rclcpp::Service<SendGoalSrv>> srv,
             const std::shared_ptr<rmw_request_id_t> header,
             const std::shared_ptr<SendGoalSrv::Request> req)
      {
        if (!cl_send_goal_->service_is_ready()) {       // (5) 快速失敗
          SendGoalSrv::Response resp;                    // accepted 預設 false
          srv->send_response(*header, resp);
          RCLCPP_WARN(logger_, "[%s轉送] 手臂端不在線 → 立即回 accepted=false",
                      prefix_.c_str());
          return;
        }
        auto fwd = std::make_shared<SendGoalSrv::Request>(*req);
        strip_goal(fwd->goal, prefix_);                  // (3) 去前綴
        RCLCPP_INFO(logger_, "[%s轉送] send_goal 押單轉發 (%zu 點)",
                    prefix_.c_str(), fwd->goal.trajectory.points.size());
        cl_send_goal_->async_send_request(
          fwd,
          [srv, header](rclcpp::Client<SendGoalSrv>::SharedFuture fut) {
            srv->send_response(*header, *fut.get());     // 原樣轉交手臂本人的回答
          });
      });                                                // ← 立即返回, 不佔 executor

    // ② cancel_goal — 零改寫
    sv_cancel_ = host_node->create_service<CancelSrv>(
      host_action + "/_action/cancel_goal",
      [this](const std::shared_ptr<rclcpp::Service<CancelSrv>> srv,
             const std::shared_ptr<rmw_request_id_t> header,
             const std::shared_ptr<CancelSrv::Request> req)
      {
        if (!cl_cancel_->service_is_ready()) {
          CancelSrv::Response resp;
          resp.return_code = CancelSrv::Response::ERROR_REJECTED;
          srv->send_response(*header, resp);
          return;
        }
        RCLCPP_WARN(logger_, "[%s轉送] cancel 押單轉發", prefix_.c_str());
        cl_cancel_->async_send_request(
          std::make_shared<CancelSrv::Request>(*req),
          [srv, header, this](rclcpp::Client<CancelSrv>::SharedFuture fut) {
            srv->send_response(*header, *fut.get());
            RCLCPP_WARN(logger_, "[%s轉送] cancel 回覆已轉交 (code=%d, 取消中 %zu 筆)",
                        prefix_.c_str(), static_cast<int>(fut.get()->return_code),
                        fut.get()->goals_canceling.size());
          });
      });

    // ③ get_result — 零改寫; 押單時長 = 整個執行期（正常, 不佔執行緒）
    sv_get_result_ = host_node->create_service<GetResultSrv>(
      host_action + "/_action/get_result",
      [this](const std::shared_ptr<rclcpp::Service<GetResultSrv>> srv,
             const std::shared_ptr<rmw_request_id_t> header,
             const std::shared_ptr<GetResultSrv::Request> req)
      {
        if (!cl_get_result_->service_is_ready()) {
          GetResultSrv::Response resp;                   // status 預設 UNKNOWN
          srv->send_response(*header, resp);
          return;
        }
        RCLCPP_INFO(logger_, "[%s轉送] get_result 押單轉發", prefix_.c_str());
        cl_get_result_->async_send_request(
          std::make_shared<GetResultSrv::Request>(*req),
          [srv, header, this](rclcpp::Client<GetResultSrv>::SharedFuture fut) {
            srv->send_response(*header, *fut.get());     // 押了整個執行期後補答
            RCLCPP_INFO(logger_, "[%s轉送] result 已轉交 (status=%d)",
                        prefix_.c_str(), static_cast<int>(fut.get()->status));
          });
      });

    // ---- 兩個 topic 的轉送 ----
    // ④ feedback — 唯一要改寫的回程內容（加前綴）; goal_id 直通
    pub_feedback_ = host_node->create_publisher<FeedbackMsg>(
      host_action + "/_action/feedback", rclcpp::QoS(10));
    sub_feedback_ = arm_node->create_subscription<FeedbackMsg>(
      arm_action + "/_action/feedback", rclcpp::QoS(10),
      [this](FeedbackMsg::ConstSharedPtr msg) {
        FeedbackMsg out = *msg;
        for (auto & n : out.feedback.joint_names) {n = prefix_ + n;}
        pub_feedback_->publish(out);
      });

    // ⑤ status — 零改寫; (4) transient_local QoS 兩側都要鏡像
    const auto status_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    pub_status_ = host_node->create_publisher<StatusMsg>(
      host_action + "/_action/status", status_qos);
    sub_status_ = arm_node->create_subscription<StatusMsg>(
      arm_action + "/_action/status", status_qos,
      [this](StatusMsg::ConstSharedPtr msg) {pub_status_->publish(*msg);});

    RCLCPP_INFO(logger_, "[%s轉送] 五端點就位: %s ↔ %s",
                prefix_.c_str(), host_action.c_str(), arm_action.c_str());
  }

private:
  std::string prefix_;
  rclcpp::Logger logger_;

  rclcpp::Client<SendGoalSrv>::SharedPtr cl_send_goal_;
  rclcpp::Client<CancelSrv>::SharedPtr cl_cancel_;
  rclcpp::Client<GetResultSrv>::SharedPtr cl_get_result_;

  rclcpp::Service<SendGoalSrv>::SharedPtr sv_send_goal_;
  rclcpp::Service<CancelSrv>::SharedPtr sv_cancel_;
  rclcpp::Service<GetResultSrv>::SharedPtr sv_get_result_;

  rclcpp::Publisher<FeedbackMsg>::SharedPtr pub_feedback_;
  rclcpp::Publisher<StatusMsg>::SharedPtr pub_status_;
  rclcpp::Subscription<FeedbackMsg>::SharedPtr sub_feedback_;
  rclcpp::Subscription<StatusMsg>::SharedPtr sub_status_;
};

size_t arg_or(int argc, char ** argv, int idx, size_t fallback)
{
  return (argc > idx) ? static_cast<size_t>(std::strtoul(argv[idx], nullptr, 10)) : fallback;
}

}  // namespace

int main(int argc, char ** argv)
{
  const size_t host_d = arg_or(argc, argv, 1, 10);
  const size_t arm_a_d = arg_or(argc, argv, 2, 11);
  const size_t arm_b_d = arg_or(argc, argv, 3, 12);

  rclcpp::install_signal_handlers();

  DomainNode host = make_domain_node("trajectory_downlink_endpoint_relay", host_d, true);
  DomainNode armA = make_domain_node("endpoint_relay_arm_a", arm_a_d, false);
  DomainNode armB = make_domain_node("endpoint_relay_arm_b", arm_b_d, false);

  ArmEndpointRelay relay_a(
    host.node, armA.node,
    "/big_arm/joint_trajectory_controller/follow_joint_trajectory", "big_");
  ArmEndpointRelay relay_b(
    host.node, armB.node,
    "/small_arm/joint_trajectory_controller/follow_joint_trajectory", "small_");

  RCLCPP_INFO(host.node->get_logger(),
    "端點透明轉送啟動: D%zu 假端點 ×2 ↔ D%zu/D%zu 真 JTC | 不替人作答, UUID 直通, "
    "失敗語意=直連", host_d, arm_a_d, arm_b_d);

  host.start();
  armA.start();
  armB.start();
  host.join();
  armA.join();
  armB.join();

  rclcpp::uninstall_signal_handlers();
  return 0;
}
