#include <tui_status/renderer.hpp>
#include <tui_status/tui_node.hpp>

#include <chrono>
#include <memory>
#include <thread>

#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto renderer = std::make_shared<tui_status::Renderer>();
  if (!renderer->initialized()) {
    rclcpp::shutdown();
    return 1;
  }

  auto node = std::make_shared<tui_status::TuiNode>(renderer);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  while (rclcpp::ok() && !node->shouldExit()) {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  executor.remove_node(node);
  rclcpp::shutdown();
  return 0;
}
