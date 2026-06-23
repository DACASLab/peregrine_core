#include <memory>
#include <stdexcept>

#include <behaviortree_ros2/tree_execution_server.hpp>
#include <behaviortree_cpp/loggers/bt_cout_logger.h>

#include "peregrine_bt/register_nodes.hpp"

class PeregrineTreeServer : public BT::TreeExecutionServer
{
public:
  PeregrineTreeServer(const rclcpp::NodeOptions& options)
  : BT::TreeExecutionServer(options)
  {
    // Load plugins + tree XMLs synchronously here instead of letting the base
    // class do it lazily from its wall timer. If a tree fails to parse or a
    // plugin is missing, the base would throw from inside the executor *after*
    // the execute_tree action was already advertised, killing the node with a
    // bare std::terminate and leaving clients to time out on "action server not
    // available". Doing it here surfaces the failure immediately, with a clear
    // message, and lets main() abort cleanly before anything is advertised.
    executeRegistration();

    const auto trees = factory().registeredBehaviorTrees();
    if (trees.empty())
    {
      throw std::runtime_error(
          "no behavior trees were loaded; check the 'behavior_trees' parameter "
          "paths and that peregrine_bt is installed");
    }
    RCLCPP_INFO(node()->get_logger(), "Loaded %zu behavior tree(s):", trees.size());
    for (const auto& name : trees)
    {
      RCLCPP_INFO(node()->get_logger(), "  - %s", name.c_str());
    }
  }

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
  std::shared_ptr<PeregrineTreeServer> server;
  try
  {
    server = std::make_shared<PeregrineTreeServer>(options);
  }
  catch (const std::exception& ex)
  {
    RCLCPP_FATAL(rclcpp::get_logger("peregrine_tree_server"),
                 "Behavior tree server failed to start: %s. The execute_tree "
                 "action will NOT be available.", ex.what());
    rclcpp::shutdown();
    return 1;
  }

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
