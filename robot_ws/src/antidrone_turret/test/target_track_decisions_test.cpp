#include <gtest/gtest.h>

#include "antidrone_turret/target_track_decisions.hpp"

namespace {

using antidrone_turret::ActuatorState;
using antidrone_turret::ActuatorStatus;
using antidrone_turret::Confidence;
using antidrone_turret::DecisionConfig;
using antidrone_turret::GimbalCommand;
using antidrone_turret::Lock;
using antidrone_turret::ScreenResolutionConfig;
using antidrone_turret::ServoCommand;
using antidrone_turret::TargetInput;
using antidrone_turret::Trigger;

constexpr DecisionConfig kConfig{0.8F, 30.0F};
constexpr ActuatorStatus kReady{ActuatorState::kReady, 0};
constexpr ActuatorStatus kReloading{ActuatorState::kReloading, 1};

TargetInput make_target(bool visible, float distance_m, float confidence)
{
  return {.visible = visible, .x = 320.0F, .y = 240.0F, .distance_m = distance_m, .confidence = confidence};
}

TEST(DecideTest, NotVisibleIsIdleSkip)
{
  const auto state = antidrone_turret::decide(make_target(false, 10.0F, 0.99F), kConfig, kReady);

  EXPECT_EQ(state.confidence, Confidence::NONE);
  EXPECT_EQ(state.lock, Lock::IDLE);
  EXPECT_EQ(state.trigger, Trigger::SKIP);
}

TEST(DecideTest, LowConfidenceIsIdleSkip)
{
  const auto state = antidrone_turret::decide(make_target(true, 10.0F, 0.5F), kConfig, kReady);

  EXPECT_EQ(state.confidence, Confidence::LOW);
  EXPECT_EQ(state.lock, Lock::IDLE);
  EXPECT_EQ(state.trigger, Trigger::SKIP);
}

TEST(DecideTest, ConfidenceExactlyAtThresholdIsNotLow)
{
  const auto state = antidrone_turret::decide(make_target(true, 10.0F, 0.8F), kConfig, kReady);

  EXPECT_EQ(state.confidence, Confidence::HIGH);
  EXPECT_EQ(state.lock, Lock::TRACK);
  EXPECT_EQ(state.trigger, Trigger::REQUESTED);
}

TEST(DecideTest, InRangeAndReadyRequestsShot)
{
  const auto state = antidrone_turret::decide(make_target(true, 20.0F, 0.9F), kConfig, kReady);

  EXPECT_EQ(state.confidence, Confidence::HIGH);
  EXPECT_EQ(state.lock, Lock::TRACK);
  EXPECT_EQ(state.trigger, Trigger::REQUESTED);
}

TEST(DecideTest, DistanceExactlyAtMaxIsInRange)
{
  const auto state = antidrone_turret::decide(make_target(true, 30.0F, 0.9F), kConfig, kReady);

  EXPECT_EQ(state.confidence, Confidence::HIGH);
  EXPECT_EQ(state.lock, Lock::TRACK);
  EXPECT_EQ(state.trigger, Trigger::REQUESTED);
}

TEST(DecideTest, OutOfRangeTracksButSkips)
{
  const auto state = antidrone_turret::decide(make_target(true, 50.0F, 0.9F), kConfig, kReady);

  EXPECT_EQ(state.confidence, Confidence::HIGH);
  EXPECT_EQ(state.lock, Lock::TRACK);
  EXPECT_EQ(state.trigger, Trigger::SKIP);
}

TEST(DecideTest, ReloadingPreemptsInRangeShot)
{
  const auto state = antidrone_turret::decide(make_target(true, 20.0F, 0.9F), kConfig, kReloading);

  EXPECT_EQ(state.confidence, Confidence::HIGH);
  EXPECT_EQ(state.lock, Lock::TRACK);
  EXPECT_EQ(state.trigger, Trigger::RELOADING);
}

TEST(DecideTest, OutOfRangeWhileReloadingSkips)
{
  const auto state = antidrone_turret::decide(make_target(true, 50.0F, 0.9F), kConfig, kReloading);

  EXPECT_EQ(state.confidence, Confidence::HIGH);
  EXPECT_EQ(state.lock, Lock::TRACK);
  EXPECT_EQ(state.trigger, Trigger::SKIP);
}

TEST(DecideTest, LowConfidenceTakesPrecedenceOverReloading)
{
  const auto state = antidrone_turret::decide(make_target(true, 20.0F, 0.5F), kConfig, kReloading);

  EXPECT_EQ(state.confidence, Confidence::LOW);
  EXPECT_EQ(state.lock, Lock::IDLE);
  EXPECT_EQ(state.trigger, Trigger::SKIP);
}

TEST(ServoParametersTest, TargetLeftOfCentreCommandsLeft)
{
  const auto servo = antidrone_turret::calculate_servo_parameters(ScreenResolutionConfig{}, 100.0F);

  EXPECT_EQ(servo.command, ServoCommand::LEFT);
  EXPECT_FLOAT_EQ(servo.target_x, 100.0F);
  EXPECT_FLOAT_EQ(servo.error_x, -220.0F);
}

TEST(ServoParametersTest, TargetRightOfCentreCommandsRight)
{
  const auto servo = antidrone_turret::calculate_servo_parameters(ScreenResolutionConfig{}, 500.0F);

  EXPECT_EQ(servo.command, ServoCommand::RIGHT);
  EXPECT_FLOAT_EQ(servo.error_x, 180.0F);
}

TEST(ServoParametersTest, TargetOnCentreCommandsCentre)
{
  const auto servo = antidrone_turret::calculate_servo_parameters(ScreenResolutionConfig{}, 320.0F);

  EXPECT_EQ(servo.command, ServoCommand::CENTER);
  EXPECT_FLOAT_EQ(servo.error_x, 0.0F);
}

TEST(GimbalParametersTest, TargetAboveCentreCommandsUp)
{
  const auto gimbal = antidrone_turret::calculate_gimbal_parameters(ScreenResolutionConfig{}, 100.0F);

  EXPECT_EQ(gimbal.command, GimbalCommand::UP);
  EXPECT_FLOAT_EQ(gimbal.target_y, 100.0F);
  EXPECT_FLOAT_EQ(gimbal.error_y, 140.0F);
}

TEST(GimbalParametersTest, TargetBelowCentreCommandsDown)
{
  const auto gimbal = antidrone_turret::calculate_gimbal_parameters(ScreenResolutionConfig{}, 400.0F);

  EXPECT_EQ(gimbal.command, GimbalCommand::DOWN);
  EXPECT_FLOAT_EQ(gimbal.error_y, -160.0F);
}

TEST(GimbalParametersTest, UsesHeightNotWidthForVerticalError)
{
  const auto gimbal = antidrone_turret::calculate_gimbal_parameters(ScreenResolutionConfig{}, 240.0F);

  EXPECT_EQ(gimbal.command, GimbalCommand::CENTER);
  EXPECT_FLOAT_EQ(gimbal.error_y, 0.0F);
}

}  // namespace
