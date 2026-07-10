// 多 domain context 共用樣板（上行橋與下行 client 共用）
// 核心技術: rclcpp 一個程序開多個 Context, 各掛不同 ROS_DOMAIN
//   (InitOptions::set_domain_id, Humble 支援)
// 規則: 節點不可跨 context 掛同一 executor → 一 context 一 executor 一執行緒
#ifndef DUAL_ARM_DOMAIN_BRIDGE__MULTI_CONTEXT_HPP_
#define DUAL_ARM_DOMAIN_BRIDGE__MULTI_CONTEXT_HPP_

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>
#include <thread>

namespace dual_arm_domain_bridge
{

// 一個 domain 的「context + node + executor + spin 執行緒」四件組
struct DomainNode
{
  std::shared_ptr<rclcpp::Context> context;
  rclcpp::Node::SharedPtr node;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
  std::thread spin_thread;

  void start()
  {
    spin_thread = std::thread([this]() {executor->spin();});
  }

  void join()
  {
    if (spin_thread.joinable()) {spin_thread.join();}
  }
};

// 建立掛在指定 DOMAIN 上的節點
// first_context = true 時負責初始化 logging（整個程序只能初始化一次）
inline DomainNode make_domain_node(
  const std::string & node_name, size_t domain_id, bool first_context)
{
  DomainNode d;

  rclcpp::InitOptions init_opts;
  init_opts.auto_initialize_logging(first_context);
  init_opts.set_domain_id(domain_id);     // [關鍵] 此 context 的 DDS domain
  init_opts.shutdown_on_signal = true;    // Ctrl+C → 關閉 context → executor 返回

  d.context = std::make_shared<rclcpp::Context>();
  d.context->init(0, nullptr, init_opts);

  rclcpp::NodeOptions node_opts;
  node_opts.context(d.context);
  d.node = std::make_shared<rclcpp::Node>(node_name, node_opts);

  rclcpp::ExecutorOptions exec_opts;
  exec_opts.context = d.context;
  d.executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>(exec_opts);
  d.executor->add_node(d.node);
  return d;
}

}  // namespace dual_arm_domain_bridge

#endif  // DUAL_ARM_DOMAIN_BRIDGE__MULTI_CONTEXT_HPP_
