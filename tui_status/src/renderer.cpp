#include <tui_status/renderer.hpp>

#include <ncurses.h>

#include <algorithm>
#include <cstdio>
#include <cctype>
#include <string>

namespace tui_status
{
namespace
{

constexpr short kColorPairLabel = 1;
constexpr short kColorPairGood = 2;
constexpr short kColorPairWarn = 3;
constexpr short kColorPairBad = 4;
constexpr short kColorPairBorder = 5;
constexpr short kColorPairTitle = 6;
constexpr short kColorPairValue = 7;
constexpr short kColorPairDim = 8;
constexpr short kColorPairAccent = 9;

constexpr int kMinRows = 24;
constexpr int kMinCols = 90;

short severityColorPair(const AlertSeverity severity)
{
  switch (severity) {
    case AlertSeverity::Info:
      return kColorPairLabel;
    case AlertSeverity::Warning:
      return kColorPairWarn;
    case AlertSeverity::Error:
      return kColorPairBad;
    default:
      return kColorPairLabel;
  }
}

short levelToColorPair(const uint8_t level)
{
  if (level >= 2) {
    return kColorPairBad;
  }
  if (level == 1) {
    return kColorPairWarn;
  }
  return kColorPairGood;
}

short boolColorPair(const bool flag)
{
  return flag ? kColorPairGood : kColorPairBad;
}

short boolWarnColorPair(const bool flag)
{
  return flag ? kColorPairGood : kColorPairWarn;
}

std::string upperCopy(std::string text)
{
  std::transform(
    text.begin(), text.end(), text.begin(),
    [](unsigned char c) {return static_cast<char>(std::toupper(c));});
  return text;
}

short stateColorPair(const StatusSnapshot & snapshot)
{
  const std::string state = upperCopy(snapshot.state);
  if (snapshot.failsafe || state == "EMERGENCY") {
    return kColorPairBad;
  }
  if (
    state == "TAKING_OFF" || state == "HOVERING" ||
    state == "FLYING" || state == "LANDING")
  {
    return kColorPairGood;
  }
  if (state == "ARMED") {
    return kColorPairWarn;
  }
  return kColorPairLabel;
}

short modeColorPair(const StatusSnapshot & snapshot)
{
  const std::string mode = upperCopy(snapshot.mode);
  if (snapshot.failsafe || mode.find("FAILSAFE") != std::string::npos || mode.find("EMERGENCY") != std::string::npos) {
    return kColorPairBad;
  }
  // Green is reserved for the active control mode (offboard healthy path).
  if (snapshot.offboard || mode.find("OFFBOARD") != std::string::npos) {
    return kColorPairGood;
  }
  return kColorPairWarn;
}

short batteryColorPair(const float battery_percent)
{
  if (battery_percent < 0.0F) {
    return kColorPairDim;
  }
  if (battery_percent < 0.20F) {
    return kColorPairBad;
  }
  if (battery_percent < 0.40F) {
    return kColorPairWarn;
  }
  return kColorPairGood;
}

short gpsColorPair(const uint8_t fix_type)
{
  if (fix_type >= 3U) {
    return kColorPairGood;
  }
  if (fix_type >= 2U) {
    return kColorPairWarn;
  }
  return kColorPairBad;
}

void withColor(const bool has_colors, const short color_pair, const chtype attrs, const bool enable)
{
  if (!has_colors) {
    if (enable) {
      attron(attrs);
    } else {
      attroff(attrs);
    }
    return;
  }

  if (enable) {
    attron(COLOR_PAIR(color_pair) | attrs);
  } else {
    attroff(COLOR_PAIR(color_pair) | attrs);
  }
}

void printLabel(const bool has_colors, const int row, const int col, const std::string & text)
{
  withColor(has_colors, kColorPairLabel, A_BOLD, true);
  mvaddnstr(row, col, text.c_str(), static_cast<int>(text.size()));
  withColor(has_colors, kColorPairLabel, A_BOLD, false);
}

void printValue(const bool has_colors, const int row, const int col, const std::string & text)
{
  withColor(has_colors, kColorPairValue, A_NORMAL, true);
  mvaddnstr(row, col, text.c_str(), static_cast<int>(text.size()));
  withColor(has_colors, kColorPairValue, A_NORMAL, false);
}

void printDim(const bool has_colors, const int row, const int col, const std::string & text)
{
  withColor(has_colors, kColorPairDim, A_DIM, true);
  mvaddnstr(row, col, text.c_str(), static_cast<int>(text.size()));
  withColor(has_colors, kColorPairDim, A_DIM, false);
}

void printBadge(
  const bool has_colors, const int row, const int col, const short color_pair, const std::string & text,
  const chtype attrs = A_BOLD)
{
  const std::string badge = " " + text + " ";
  withColor(has_colors, color_pair, attrs, true);
  mvaddnstr(row, col, badge.c_str(), static_cast<int>(badge.size()));
  withColor(has_colors, color_pair, attrs, false);
}

void drawPanel(
  const bool has_colors, const int y, const int x, const int h, const int w, const std::string & title)
{
  if (h < 3 || w < 6) {
    return;
  }

  withColor(has_colors, kColorPairBorder, A_NORMAL, true);
  mvhline(y, x + 1, ACS_HLINE, w - 2);
  mvhline(y + h - 1, x + 1, ACS_HLINE, w - 2);
  mvvline(y + 1, x, ACS_VLINE, h - 2);
  mvvline(y + 1, x + w - 1, ACS_VLINE, h - 2);
  mvaddch(y, x, ACS_ULCORNER);
  mvaddch(y, x + w - 1, ACS_URCORNER);
  mvaddch(y + h - 1, x, ACS_LLCORNER);
  mvaddch(y + h - 1, x + w - 1, ACS_LRCORNER);
  withColor(has_colors, kColorPairBorder, A_NORMAL, false);

  const std::string header = " " + title + " ";
  withColor(has_colors, kColorPairTitle, A_BOLD, true);
  mvaddnstr(y, x + 2, header.c_str(), std::max(0, w - 4));
  withColor(has_colors, kColorPairTitle, A_BOLD, false);
}

void lineAt(const int row, const int col, const int width)
{
  if (width > 0) {
    mvhline(row, col, ACS_HLINE, width);
  }
}

void formatTo(char * out, const std::size_t out_size, const char * fmt, ...)
{
  if (out == nullptr || out_size == 0U) {
    return;
  }

  va_list args;
  va_start(args, fmt);
  vsnprintf(out, out_size, fmt, args);
  va_end(args);
}

}  // namespace

Renderer::Renderer()
{
  if (initscr() == nullptr) {
    initialized_ = false;
    return;
  }

  cbreak();
  noecho();
  curs_set(0);
  keypad(stdscr, TRUE);
  nodelay(stdscr, TRUE);

  hasColors_ = has_colors();
  if (hasColors_) {
    start_color();
    use_default_colors();
    init_pair(kColorPairLabel, COLOR_CYAN, -1);
    init_pair(kColorPairGood, COLOR_GREEN, -1);
    init_pair(kColorPairWarn, COLOR_YELLOW, -1);
    init_pair(kColorPairBad, COLOR_RED, -1);
    init_pair(kColorPairBorder, COLOR_BLUE, -1);
    init_pair(kColorPairTitle, COLOR_WHITE, -1);
    init_pair(kColorPairValue, COLOR_WHITE, -1);
    init_pair(kColorPairDim, COLOR_BLUE, -1);
    init_pair(kColorPairAccent, COLOR_MAGENTA, -1);
  }

  initialized_ = true;
}

Renderer::~Renderer()
{
  if (initialized_) {
    endwin();
  }
}

int Renderer::pollKey() const
{
  if (!initialized_) {
    return ERR;
  }
  return getch();
}

void Renderer::render(
  const StatusSnapshot & snapshot,
  const std::vector<AlertEntry> & alerts,
  const std::size_t alert_scroll,
  const std::string & uav_namespace) const
{
  if (!initialized_) {
    return;
  }

  erase();

  int rows = 0;
  int cols = 0;
  getmaxyx(stdscr, rows, cols);

  if (rows < kMinRows || cols < kMinCols) {
    withColor(hasColors_, kColorPairBad, A_BOLD, true);
    mvprintw(0, 0, "Terminal too small (%dx%d). Need at least %dx%d.", cols, rows, kMinCols, kMinRows);
    withColor(hasColors_, kColorPairBad, A_BOLD, false);
    mvprintw(2, 0, "Tip: enlarge terminal for full panel layout.");
    refresh();
    return;
  }

  withColor(hasColors_, kColorPairBorder, A_NORMAL, true);
  box(stdscr, 0, 0);
  withColor(hasColors_, kColorPairBorder, A_NORMAL, false);

  const std::string ns = uav_namespace.empty() ? std::string("/") : uav_namespace;
  const std::string title = " PEREGRINE FLIGHT CONSOLE ";
  const std::string subtitle = " Namespace: " + ns + " ";
  withColor(hasColors_, kColorPairTitle, A_BOLD, true);
  mvaddnstr(0, 2, title.c_str(), cols - 4);
  withColor(hasColors_, kColorPairTitle, A_BOLD, false);
  withColor(hasColors_, kColorPairAccent, A_BOLD, true);
  mvaddnstr(1, 2, subtitle.c_str(), cols - 4);
  withColor(hasColors_, kColorPairAccent, A_BOLD, false);
  withColor(hasColors_, kColorPairBorder, A_NORMAL, true);
  lineAt(2, 1, cols - 2);
  withColor(hasColors_, kColorPairBorder, A_NORMAL, false);

  const int content_x = 2;
  const int content_w = cols - 4;
  const int top_y = 3;
  const int top_h = 5;
  const int middle_y = top_y + top_h;
  const int middle_h = 8;
  const int alerts_y = middle_y + middle_h;
  const int alerts_h = rows - alerts_y - 2;
  const int left_w = content_w / 2 - 1;
  const int right_x = content_x + left_w + 1;
  const int right_w = content_w - left_w - 1;

  drawPanel(hasColors_, top_y, content_x, top_h, content_w, "Flight");
  drawPanel(hasColors_, middle_y, content_x, middle_h, left_w, "Managers / Readiness");
  drawPanel(hasColors_, middle_y, right_x, middle_h, right_w, "Safety / Hardware");
  drawPanel(hasColors_, alerts_y, content_x, alerts_h, content_w, "Alerts");

  int row = top_y + 1;
  int col = content_x + 2;
  const int top_badge_col_start = content_x + content_w - 33;
  printLabel(hasColors_, row, col, "State:");
  printBadge(hasColors_, row, col + 7, stateColorPair(snapshot), snapshot.state);

  printLabel(hasColors_, row, col + 28, "Mode:");
  const int mode_col = col + 34;
  const int mode_width = std::max(6, top_badge_col_start - mode_col - 2);
  const std::string mode_text = snapshot.mode.empty() ? std::string("UNKNOWN") : truncate(snapshot.mode, mode_width);
  printBadge(hasColors_, row, mode_col, modeColorPair(snapshot), mode_text, A_BOLD);

  int badge_col = top_badge_col_start;
  printBadge(
    hasColors_, row, badge_col, snapshot.armed ? kColorPairAccent : kColorPairLabel,
    snapshot.armed ? "ARM" : "DIS", snapshot.armed ? A_BOLD : A_DIM);
  badge_col += 6;
  printBadge(
    hasColors_, row, badge_col, boolWarnColorPair(snapshot.offboard),
    snapshot.offboard ? "OFF" : "MAN", snapshot.offboard ? A_BOLD : A_DIM);
  badge_col += 6;
  printBadge(
    hasColors_, row, badge_col, boolColorPair(snapshot.connected),
    snapshot.connected ? "LINK" : "NOLINK", snapshot.connected ? A_DIM : A_BOLD);
  badge_col += 9;
  printBadge(
    hasColors_, row, badge_col, snapshot.failsafe ? kColorPairBad : kColorPairGood,
    snapshot.failsafe ? "FAIL" : "OK", snapshot.failsafe ? A_BOLD : A_DIM);

  row = top_y + 2;
  if (snapshot.has_pose) {
    char buffer[128];
    formatTo(buffer, sizeof(buffer), "Pos[m] X:%7.2f  Y:%7.2f  Z:%7.2f", snapshot.x_m, snapshot.y_m, snapshot.z_m);
    printValue(hasColors_, row, col, buffer);
  } else {
    printDim(hasColors_, row, col, "Position waiting for estimated_state...");
  }

  row = top_y + 3;
  if (snapshot.has_velocity) {
    char vel[128];
    formatTo(
      vel, sizeof(vel), "Vel[m/s] VX:%6.2f  VY:%6.2f  VZ:%6.2f",
      snapshot.vx_mps, snapshot.vy_mps, snapshot.vz_mps);
    printValue(hasColors_, row, col, vel);
    char rpy[128];
    formatTo(
      rpy, sizeof(rpy), "RPY[deg] R:%6.1f  P:%6.1f  Y:%6.1f",
      snapshot.roll_deg, snapshot.pitch_deg, snapshot.yaw_deg);
    printValue(hasColors_, row, col + 42, rpy);
  } else {
    printDim(hasColors_, row, col, "Velocity waiting for estimated_state...");
  }

  row = middle_y + 1;
  col = content_x + 2;
  printLabel(hasColors_, row, col, "EST:");
  printValue(hasColors_, row, col + 5, truncate(snapshot.estimation_module, left_w - 20));
  printBadge(hasColors_, row, col + left_w - 16, boolColorPair(snapshot.estimation_healthy), snapshot.estimation_healthy ? "OK" : "BAD");

  row = middle_y + 2;
  printLabel(hasColors_, row, col, "CTL:");
  printValue(hasColors_, row, col + 5, truncate(snapshot.control_module, left_w - 20));
  printBadge(hasColors_, row, col + left_w - 16, boolColorPair(snapshot.control_healthy), snapshot.control_healthy ? "OK" : "BAD");

  row = middle_y + 3;
  printLabel(hasColors_, row, col, "TRJ:");
  printValue(hasColors_, row, col + 5, truncate(snapshot.trajectory_module, left_w - 20));
  printBadge(
    hasColors_, row, col + left_w - 16, boolColorPair(snapshot.trajectory_healthy),
    snapshot.trajectory_healthy ? "OK" : "BAD");

  row = middle_y + 5;
  printLabel(hasColors_, row, col, "Deps:");
  printBadge(
    hasColors_, row, col + 6, boolWarnColorPair(snapshot.dependencies_ready),
    snapshot.dependencies_ready ? "READY" : "BLOCKED");
  printLabel(hasColors_, row, col + 16, "Safety:");
  if (snapshot.has_safety_status) {
    printBadge(
      hasColors_, row, col + 24, boolWarnColorPair(snapshot.safety_ready),
      snapshot.safety_ready ? "READY" : "BLOCKED");
  } else {
    printBadge(hasColors_, row, col + 24, kColorPairLabel, "OFF", A_DIM);
  }

  row = middle_y + 1;
  col = right_x + 2;
  printLabel(hasColors_, row, col, "Safety:");
  if (snapshot.has_safety_status) {
    printBadge(
      hasColors_, row, col + 8, levelToColorPair(snapshot.safety_level),
      "L" + std::to_string(snapshot.safety_level));
    printValue(hasColors_, row, col + 13, truncate(snapshot.safety_reason, right_w - 16));
  } else {
    printBadge(hasColors_, row, col + 8, kColorPairLabel, "OFF", A_DIM);
    printDim(hasColors_, row, col + 13, "not enabled in this launch");
  }

  row = middle_y + 2;
  printLabel(hasColors_, row, col, "PX4:");
  printBadge(
    hasColors_, row, col + 5, boolColorPair(snapshot.connected),
    snapshot.connected ? "CONNECTED" : "DISCONNECTED");
  printBadge(
    hasColors_, row, col + 18, snapshot.failsafe ? kColorPairBad : kColorPairGood,
    snapshot.failsafe ? "FAILSAFE" : "NOMINAL");

  row = middle_y + 3;
  printLabel(hasColors_, row, col, "Battery:");
  if (snapshot.battery_percent >= 0.0F) {
    char batt[96];
    formatTo(
      batt, sizeof(batt), "%5.1f%%  %.2fV",
      snapshot.battery_percent * 100.0F, snapshot.battery_voltage);
    printBadge(hasColors_, row, col + 9, batteryColorPair(snapshot.battery_percent), batt);
  } else {
    printDim(hasColors_, row, col + 9, "n/a");
  }

  row = middle_y + 4;
  printLabel(hasColors_, row, col, "GPS:");
  char gps[96];
  formatTo(
    gps, sizeof(gps), "fix=%u  sats=%u  hdop=%.2f",
    snapshot.gps_fix_type, snapshot.gps_satellites, snapshot.gps_hdop);
  printBadge(hasColors_, row, col + 5, gpsColorPair(snapshot.gps_fix_type), gps);

  row = middle_y + 6;
  printLabel(hasColors_, row, col, "Checkers:");
  if (snapshot.checker_levels.empty()) {
    if (snapshot.has_safety_status) {
      printDim(hasColors_, row, col + 10, "no checker results yet");
    } else {
      printDim(hasColors_, row, col + 10, "safety monitor not active");
    }
  } else {
    const int line_width = right_w - 13;
    std::string checker_line;
    for (std::size_t i = 0; i < snapshot.checker_levels.size(); ++i) {
      const auto & checker = snapshot.checker_levels[i];
      const std::string token = checker.name + ":L" + std::to_string(checker.level);
      if (!checker_line.empty()) {
        checker_line += " | ";
      }
      checker_line += token;
    }
    printValue(hasColors_, row, col + 10, truncate(checker_line, line_width));
  }

  const int alert_title_row = alerts_y + 1;
  printLabel(hasColors_, alert_title_row, content_x + 2, "Newest first");
  char alert_meta[96];
  formatTo(alert_meta, sizeof(alert_meta), "count=%zu  scroll=%zu", alerts.size(), alert_scroll);
  printDim(hasColors_, alert_title_row, content_x + content_w - 30, alert_meta);

  const int alertRows = std::max(0, alerts_h - 3);
  const std::size_t start = std::min(alert_scroll, alerts.size());
  for (int i = 0; i < alertRows; ++i) {
    const std::size_t idx = start + static_cast<std::size_t>(i);
    if (idx >= alerts.size()) {
      break;
    }

    const auto & entry = alerts[idx];
    const int alert_row = alerts_y + 2 + i;
    const std::string severity = severityToString(entry.severity);
    printBadge(hasColors_, alert_row, content_x + 2, severityColorPair(entry.severity), severity);

    const std::string line = entry.timestamp + "  " + entry.message;
    printValue(hasColors_, alert_row, content_x + 10, truncate(line, content_w - 12));
  }

  withColor(hasColors_, kColorPairBorder, A_NORMAL, true);
  lineAt(rows - 3, 1, cols - 2);
  withColor(hasColors_, kColorPairBorder, A_NORMAL, false);
  printDim(hasColors_, rows - 2, 2, "Q:Quit  C:Clear alerts  Up/Down:Scroll alerts");

  refresh();
}

std::string Renderer::truncate(const std::string & text, const int max_width)
{
  if (max_width <= 0) {
    return std::string();
  }
  if (static_cast<int>(text.size()) <= max_width) {
    return text;
  }
  if (max_width <= 3) {
    return text.substr(0, static_cast<std::size_t>(max_width));
  }
  return text.substr(0, static_cast<std::size_t>(max_width - 3)) + "...";
}

}  // namespace tui_status
