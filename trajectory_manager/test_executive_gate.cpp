#include <gtest/gtest.h>

#include <trajectory_manager/executive_gate.hpp>

TEST(ExecutiveGateTest, RejectsWithoutExecutiveState)
{
  std::optional<peregrine_interfaces::msg::UAVState> state;
  std::string reason;
  EXPECT_FALSE(trajectory_manager::executiveAllowsMotion(state, &reason));
  EXPECT_EQ(reason, "MISSING_EXECUTIVE_STATE");
}

TEST(ExecutiveGateTest, RejectsFaultLatched)
{
  peregrine_interfaces::msg::UAVState s;
  s.dependencies_ready = true;
  s.fault_latched = true;
  s.failsafe = false;
  s.motion_authorized = true;
  std::string reason;
  EXPECT_FALSE(trajectory_manager::executiveAllowsMotion(s, &reason));
  EXPECT_EQ(reason, "EXECUTIVE_FAULT_LATCHED");
}

TEST(ExecutiveGateTest, AllowsNominal)
{
  peregrine_interfaces::msg::UAVState s;
  s.dependencies_ready = true;
  s.fault_latched = false;
  s.failsafe = false;
  s.motion_authorized = true;
  std::string reason;
  EXPECT_TRUE(trajectory_manager::executiveAllowsMotion(s, &reason));
  EXPECT_EQ(reason, "OK");
}
