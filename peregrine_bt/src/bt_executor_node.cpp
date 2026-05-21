#include <memory>
#include <string>

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/bt_cout_logger.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "peregrine_bt/register_nodes.hpp"

namespace peregrine_bt
{

class BTExecutorNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit BTExecutorNode(const rclcpp::NodeOptions & options)
  : rclcpp_lifecycle::LifecycleNode("bt_executor", options),
    client_node_(std::make_shared<rclcpp::Node>(
        "bt_client", get_namespace(),
        rclcpp::NodeOptions().parameter_overrides(
            {rclcpp::Parameter("use_sim_time",
                               get_parameter("use_sim_time").as_bool())})))
  {
    declare_parameter<std::string>("tree_file", "");
    declare_parameter<double>("tick_rate_hz", 2.0);
  }

  rclcpp::Node::SharedPtr getClientNode() const { return client_node_; }

private:
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &) override
  {
    tree_file_ = get_parameter("tree_file").as_string();
    tick_rate_hz_ = get_parameter("tick_rate_hz").as_double();

    if (tree_file_.empty()) {
      RCLCPP_ERROR(get_logger(), "Parameter 'tree_file' must be set");
      return CallbackReturn::FAILURE;
    }

    factory_ = std::make_unique<BT::BehaviorTreeFactory>();
    registerAllNodes(*factory_, client_node_);

    try {
      tree_ = std::make_unique<BT::Tree>(factory_->createTreeFromFile(tree_file_));
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "Failed to load tree '%s': %s",
                   tree_file_.c_str(), e.what());
      return CallbackReturn::FAILURE;
    }

    logger_ = std::make_unique<BT::StdCoutLogger>(*tree_);

    RCLCPP_INFO(get_logger(), "Loaded tree from '%s', tick rate %.1f Hz",
                tree_file_.c_str(), tick_rate_hz_);
    return CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &) override
  {
    auto period = std::chrono::duration<double>(1.0 / tick_rate_hz_);
    tick_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() { tickTree(); });

    RCLCPP_INFO(get_logger(), "BT executor activated");
    return CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &) override
  {
    tick_timer_.reset();
    if (tree_) {
      tree_->haltTree();
    }
    RCLCPP_INFO(get_logger(), "BT executor deactivated — tree halted");
    return CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State &) override
  {
    logger_.reset();
    tree_.reset();
    factory_.reset();
    return CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State &) override
  {
    tick_timer_.reset();
    if (tree_) {
      tree_->haltTree();
    }
    logger_.reset();
    tree_.reset();
    factory_.reset();
    return CallbackReturn::SUCCESS;
  }

  void tickTree()
  {
    if (!tree_) {
      return;
    }

    auto status = tree_->tickOnce();
    if (status == BT::NodeStatus::SUCCESS) {
      RCLCPP_INFO(get_logger(), "Tree completed with SUCCESS");
      tick_timer_.reset();
    } else if (status == BT::NodeStatus::FAILURE) {
      RCLCPP_WARN(get_logger(), "Tree completed with FAILURE");
      tick_timer_.reset();
    }
  }

  rclcpp::Node::SharedPtr client_node_;
  std::string tree_file_;
  double tick_rate_hz_{2.0};
  std::unique_ptr<BT::BehaviorTreeFactory> factory_;
  std::unique_ptr<BT::Tree> tree_;
  std::unique_ptr<BT::StdCoutLogger> logger_;
  rclcpp::TimerBase::SharedPtr tick_timer_;
};

}  // namespace peregrine_bt

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<peregrine_bt::BTExecutorNode>(rclcpp::NodeOptions());

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.add_node(node->getClientNode());

  executor.spin();
  rclcpp::shutdown();
  return 0;
}
