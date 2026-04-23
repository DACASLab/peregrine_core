#include <mission_executor/mission_executor_node.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<mission_executor::MissionExecutorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
