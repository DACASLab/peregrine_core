#include <behaviortree_ros2/tree_execution_server.hpp>
#include <behaviortree_cpp/loggers/bt_cout_logger.h>

#include "peregrine_bt/register_nodes.hpp"

class PeregrineTreeServer : public BT::TreeExecutionServer
{
public:
  PeregrineTreeServer(const rclcpp::NodeOptions& options)
  : BT::TreeExecutionServer(options)
  {}

  void registerNodesIntoFactory(BT::BehaviorTreeFactory& factory) override
  {
    peregrine_bt::registerAllNodes(factory, node());
  }

  void onTreeCreated(BT::Tree& tree) override
  {
    logger_ = std::make_shared<BT::StdCoutLogger>(tree);
    RCLCPP_INFO(node()->get_logger(), "Tree '%s' created", treeName().c_str());
  }

  std::optional<std::string> onTreeExecutionCompleted(BT::NodeStatus /*status*/,
                                                       bool was_cancelled) override
  {
    logger_.reset();
    if (was_cancelled)
    {
      return "Tree cancelled";
    }
    return std::nullopt;
  }

private:
  std::shared_ptr<BT::StdCoutLogger> logger_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  auto server = std::make_shared<PeregrineTreeServer>(options);

  // MultiThreadedExecutor with timeout to avoid deadlock on dynamic sub/pub
  // destruction (known rclcpp issue, same workaround as behaviortree_ros2 sample).
  rclcpp::executors::MultiThreadedExecutor exec(
      rclcpp::ExecutorOptions(), 0, false, std::chrono::milliseconds(250));
  exec.add_node(server->node());
  exec.spin();
  exec.remove_node(server->node());

  rclcpp::shutdown();
  return 0;
}
