#include <tui_status/alert_buffer.hpp>

#include <chrono>
#include <ctime>

namespace tui_status
{

AlertBuffer::AlertBuffer(const std::size_t capacity)
: capacity_(capacity == 0 ? 1 : capacity)
{
}

void AlertBuffer::push(const AlertSeverity severity, const std::string & message)
{
  std::scoped_lock lock(mutex_);

  entries_.push_front(AlertEntry{nowString(), severity, message});
  while (entries_.size() > capacity_) {
    entries_.pop_back();
  }
}

void AlertBuffer::clear()
{
  std::scoped_lock lock(mutex_);
  entries_.clear();
}

std::size_t AlertBuffer::size() const
{
  std::scoped_lock lock(mutex_);
  return entries_.size();
}

std::vector<AlertEntry> AlertBuffer::snapshot() const
{
  std::scoped_lock lock(mutex_);
  return std::vector<AlertEntry>(entries_.begin(), entries_.end());
}

std::string AlertBuffer::nowString()
{
  const auto now = std::chrono::system_clock::now();
  const std::time_t nowTime = std::chrono::system_clock::to_time_t(now);

  std::tm nowTm{};
  localtime_r(&nowTime, &nowTm);

  char buffer[16];
  std::strftime(buffer, sizeof(buffer), "%H:%M:%S", &nowTm);
  return std::string(buffer);
}

std::string severityToString(const AlertSeverity severity)
{
  switch (severity) {
    case AlertSeverity::Info:
      return " INFO ";
    case AlertSeverity::Warning:
      return " WARN ";
    case AlertSeverity::Error:
      return "ERROR";
    default:
      return "  ???  ";
  }
}

}  // namespace tui_status
