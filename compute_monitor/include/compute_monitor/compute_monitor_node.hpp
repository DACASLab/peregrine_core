#pragma once

#include <array>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <peregrine_interfaces/msg/compute_status.hpp>

namespace compute_monitor
{

class ComputeMonitorNode : public rclcpp::Node
{
public:
  explicit ComputeMonitorNode(const rclcpp::NodeOptions & options);

private:
  void onTimer();

  struct CpuSample
  {
    uint64_t idle{0};
    uint64_t total{0};
  };

  static CpuSample readCpuSample();
  static float readCpuTempC();
  static float readCpuFreqGhz();
  static std::pair<float, float> readRamGb();

  rclcpp::Publisher<peregrine_interfaces::msg::ComputeStatus>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  CpuSample prevSample_;
};

}  // namespace compute_monitor
