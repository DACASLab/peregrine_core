#include <compute_monitor/compute_monitor_node.hpp>

#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace compute_monitor
{
using namespace std::chrono_literals;

ComputeMonitorNode::ComputeMonitorNode(const rclcpp::NodeOptions & options)
: Node("compute_monitor", options)
{
  pub_ = this->create_publisher<peregrine_interfaces::msg::ComputeStatus>(
    "compute_status", rclcpp::QoS(10).reliable());

  prevSample_ = readCpuSample();

  timer_ = this->create_wall_timer(500ms, [this]() { onTimer(); });

  RCLCPP_INFO(this->get_logger(), "compute_monitor started (2 Hz)");
}

void ComputeMonitorNode::onTimer()
{
  auto msg = peregrine_interfaces::msg::ComputeStatus();
  msg.header.stamp = this->now();

  const auto sample = readCpuSample();
  const uint64_t total_diff = sample.total - prevSample_.total;
  const uint64_t idle_diff = sample.idle - prevSample_.idle;
  if (total_diff > 0) {
    msg.cpu_percent = 100.0F * static_cast<float>(total_diff - idle_diff) /
      static_cast<float>(total_diff);
  }
  prevSample_ = sample;

  msg.cpu_freq_ghz = readCpuFreqGhz();
  msg.cpu_temp_c = readCpuTempC();

  const auto [used, total] = readRamGb();
  msg.ram_used_gb = used;
  msg.ram_total_gb = total;

  pub_->publish(msg);
}

ComputeMonitorNode::CpuSample ComputeMonitorNode::readCpuSample()
{
  CpuSample result;
  std::ifstream f("/proc/stat");
  std::string line;
  if (!std::getline(f, line)) {
    return result;
  }

  // "cpu  user nice system idle iowait irq softirq steal ..."
  std::istringstream ss(line);
  std::string label;
  ss >> label;

  std::vector<uint64_t> fields;
  uint64_t val = 0;
  while (ss >> val) {
    fields.push_back(val);
  }

  for (const auto v : fields) {
    result.total += v;
  }
  // idle is field index 3, iowait is field index 4
  if (fields.size() > 4) {
    result.idle = fields[3] + fields[4];
  } else if (fields.size() > 3) {
    result.idle = fields[3];
  }

  return result;
}

float ComputeMonitorNode::readCpuTempC()
{
  // Jetson Orin: /sys/devices/virtual/thermal/thermal_zone0/temp (millidegrees)
  // Falls back through multiple zones; works on generic Linux too.
  static const char * const paths[] = {
    "/sys/devices/virtual/thermal/thermal_zone0/temp",
    "/sys/devices/virtual/thermal/thermal_zone1/temp",
    "/sys/class/thermal/thermal_zone0/temp",
  };

  for (const auto * path : paths) {
    std::ifstream f(path);
    int millideg = 0;
    if (f >> millideg) {
      return static_cast<float>(millideg) / 1000.0F;
    }
  }
  return -1.0F;
}

float ComputeMonitorNode::readCpuFreqGhz()
{
  // Average across all online cores
  float sum_khz = 0.0F;
  int count = 0;
  for (int i = 0; i < 16; ++i) {
    const std::string path =
      "/sys/devices/system/cpu/cpu" + std::to_string(i) + "/cpufreq/scaling_cur_freq";
    std::ifstream f(path);
    float khz = 0.0F;
    if (f >> khz) {
      sum_khz += khz;
      ++count;
    }
  }
  if (count == 0) {
    return -1.0F;
  }
  return sum_khz / static_cast<float>(count) / 1.0e6F;
}

std::pair<float, float> ComputeMonitorNode::readRamGb()
{
  std::ifstream f("/proc/meminfo");
  std::string line;
  float total_kb = 0.0F;
  float available_kb = 0.0F;
  bool got_total = false;
  bool got_available = false;

  while (std::getline(f, line)) {
    if (!got_total && line.rfind("MemTotal:", 0) == 0) {
      total_kb = static_cast<float>(std::atof(line.c_str() + 9));
      got_total = true;
    } else if (!got_available && line.rfind("MemAvailable:", 0) == 0) {
      available_kb = static_cast<float>(std::atof(line.c_str() + 13));
      got_available = true;
    }
    if (got_total && got_available) {
      break;
    }
  }

  constexpr float kKbToGb = 1.0F / (1024.0F * 1024.0F);
  const float total_gb = total_kb * kKbToGb;
  const float used_gb = (total_kb - available_kb) * kKbToGb;
  return {used_gb, total_gb};
}

}  // namespace compute_monitor

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(compute_monitor::ComputeMonitorNode)
