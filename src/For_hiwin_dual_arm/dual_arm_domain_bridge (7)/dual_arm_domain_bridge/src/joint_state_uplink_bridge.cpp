// 上行橋（Plan B 程式 1/2，常駐、無狀態）
// 職責 = 現有 joint_state_merger 搬到跨 domain 版 + 原廠關節名→前綴名翻譯
//
//   ctx_armA (D20): 訂 /joint_states ── joint_N → big_joint_N ──┐
//   ctx_armB (D30): 訂 /joint_states ── joint_N → small_joint_N ┤→ 合併 12 軸
//   ctx_host (D10): ────────────────────────────────────────────┘→ 發 /joint_states
//
// 消費者: move_group CurrentStateMonitor (規劃起點)、主機 rsp→TF→RViz 顯示
// 沒有它主機就「瞎」: 規劃報 Failed to fetch current robot state、RViz 模型僵住
//
// 用法: ros2 run dual_arm_domain_bridge joint_state_uplink_bridge [host_d armA_d armB_d]
//       預設 domain: host=10, armA=11, armB=12

#include "dual_arm_domain_bridge/multi_context.hpp"

#include <sensor_msgs/msg/joint_state.hpp>

#include <cstdlib>
#include <memory>
#include <mutex>
#include <string>

using dual_arm_domain_bridge::DomainNode;
using dual_arm_domain_bridge::make_domain_node;
using sensor_msgs::msg::JointState;

namespace
{

// ---- 設定（前綴對齊主機雙臂模型）----
const char * kPrefixA = "big_";     // joint_1 → big_joint_1
const char * kPrefixB = "small_";   // joint_1 → small_joint_1

struct SharedState
{
  std::mutex mtx;
  JointState arm_a;        // 已改名（加前綴）後的最新狀態
  JointState arm_b;
  bool has_a{false};
  bool has_b{false};
};

// 改名加前綴（上行的名字翻譯: 原廠 joint_N → 前綴名）
JointState add_prefix(const JointState & in, const std::string & prefix)
{
  JointState out = in;
  for (auto & n : out.name) {
    n = prefix + n;
  }
  return out;
}

// 兩臂快取 → 一筆 12 軸合併訊息（position 必併; velocity 兩臂都完整才併）
bool build_merged(SharedState & s, JointState & merged)
{
  std::lock_guard<std::mutex> lk(s.mtx);
  if (!s.has_a || !s.has_b) {return false;}

  merged.name = s.arm_a.name;
  merged.name.insert(merged.name.end(), s.arm_b.name.begin(), s.arm_b.name.end());

  merged.position = s.arm_a.position;
  merged.position.insert(merged.position.end(),
                         s.arm_b.position.begin(), s.arm_b.position.end());

  const bool vel_ok = s.arm_a.velocity.size() == s.arm_a.name.size() &&
                      s.arm_b.velocity.size() == s.arm_b.name.size();
  if (vel_ok) {
    merged.velocity = s.arm_a.velocity;
    merged.velocity.insert(merged.velocity.end(),
                           s.arm_b.velocity.begin(), s.arm_b.velocity.end());
  }
  return true;
}

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

  // 三個 context（host 先建, 負責初始化 logging）
  DomainNode host = make_domain_node("joint_state_uplink_bridge", host_d, true);
  DomainNode armA = make_domain_node("uplink_arm_a", arm_a_d, false);
  DomainNode armB = make_domain_node("uplink_arm_b", arm_b_d, false);

  auto state = std::make_shared<SharedState>();
  auto pub = host.node->create_publisher<JointState>("/joint_states", 10);
  auto host_clock = host.node->get_clock();

  // 任一臂更新 → 立刻合併發布（事件驅動, 最低延遲）
  auto publish_merged = [state, pub, host_clock]() {
      JointState merged;
      if (!build_merged(*state, merged)) {return;}
      merged.header.stamp = host_clock->now();
      pub->publish(merged);
    };

  auto sub_a = armA.node->create_subscription<JointState>(
    "/joint_states", 10,
    [state, publish_merged](JointState::ConstSharedPtr msg) {
      {
        std::lock_guard<std::mutex> lk(state->mtx);
        state->arm_a = add_prefix(*msg, kPrefixA);
        state->has_a = true;
      }
      publish_merged();
    });

  auto sub_b = armB.node->create_subscription<JointState>(
    "/joint_states", 10,
    [state, publish_merged](JointState::ConstSharedPtr msg) {
      {
        std::lock_guard<std::mutex> lk(state->mtx);
        state->arm_b = add_prefix(*msg, kPrefixB);
        state->has_b = true;
      }
      publish_merged();
    });

  RCLCPP_INFO(host.node->get_logger(),
    "上行橋啟動: D%zu(A臂,加前綴 %s) + D%zu(B臂,加前綴 %s) → D%zu /joint_states (12軸)",
    arm_a_d, kPrefixA, arm_b_d, kPrefixB, host_d);
  RCLCPP_INFO(host.node->get_logger(),
    "等待兩臂 /joint_states ... (任一臂未上線前不發布)");

  host.start();
  armA.start();
  armB.start();
  host.join();
  armA.join();
  armB.join();

  rclcpp::uninstall_signal_handlers();
  return 0;
}
