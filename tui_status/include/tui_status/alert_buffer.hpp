#pragma once

#include <cstddef>
#include <deque>
#include <mutex>
#include <string>
#include <vector>

namespace tui_status
{

enum class AlertSeverity
{
  Info,
  Warning,
  Error
};

struct AlertEntry
{
  std::string timestamp;
  AlertSeverity severity{AlertSeverity::Info};
  std::string message;
};

class AlertBuffer
{
public:
  explicit AlertBuffer(std::size_t capacity);

  void push(AlertSeverity severity, const std::string & message);
  void clear();

  std::size_t size() const;
  std::vector<AlertEntry> snapshot() const;

private:
  static std::string nowString();

  std::size_t capacity_{100};
  mutable std::mutex mutex_;
  std::deque<AlertEntry> entries_;
};

std::string severityToString(AlertSeverity severity);

}  // namespace tui_status
