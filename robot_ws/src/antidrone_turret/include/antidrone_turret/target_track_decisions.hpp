#pragma once

#include <cstdint>

#include "antidrone_turret/actuator_model.hpp"

namespace antidrone_turret {

enum class ServoCommand : int8_t { RIGHT = 1, LEFT = -1, CENTER = 0 };
enum class GimbalCommand : int8_t { UP = 1, DOWN = -1, CENTER = 0 };

enum class Confidence : int8_t { NONE = 0, LOW = 1, HIGH = 2 };
enum class Lock : int8_t { IDLE = 0, TRACK = 1 };
enum class Trigger : int8_t { SKIP = 0, REQUESTED = 1, RELOADING = 2 };

inline const char* to_string(ActuatorState state)
{
  switch (state) {
    case ActuatorState::kReady:
      return "READY";
    case ActuatorState::kReloading:
      return "RELOADING";
  }
  return "UNKNOWN";
}

inline const char* to_string(ServoCommand command)
{
  switch (command) {
    case ServoCommand::LEFT:
      return "LEFT";
    case ServoCommand::CENTER:
      return "CENTER";
    case ServoCommand::RIGHT:
      return "RIGHT";
  }
  return "UNKNOWN";
}

inline const char* to_string(GimbalCommand command)
{
  switch (command) {
    case GimbalCommand::DOWN:
      return "DOWN";
    case GimbalCommand::CENTER:
      return "CENTER";
    case GimbalCommand::UP:
      return "UP";
  }
  return "UNKNOWN";
}

inline const char* to_string(Confidence confidence)
{
  switch (confidence) {
    case Confidence::NONE:
      return "NONE";
    case Confidence::LOW:
      return "LOW";
    case Confidence::HIGH:
      return "HIGH";
  }
  return "UNKNOWN";
}

inline const char* to_string(Lock lock)
{
  switch (lock) {
    case Lock::IDLE:
      return "IDLE";
    case Lock::TRACK:
      return "TRACK";
  }
  return "UNKNOWN";
}

inline const char* to_string(Trigger trigger)
{
  switch (trigger) {
    case Trigger::SKIP:
      return "SKIP";
    case Trigger::REQUESTED:
      return "REQUESTED";
    case Trigger::RELOADING:
      return "RELOADING";
  }
  return "UNKNOWN";
}

// config yaml sourced
struct DecisionConfig {
  float confidence_threshold{0.8F};
  float max_distance_m{30.0F};
};

// config yaml sourced
struct ScreenResolutionConfig {
  float width{640.0F};
  float height{480.0F};
};

struct TargetInput {
  bool visible;
  float x;
  float y;
  float distance_m;
  float confidence;
};

struct ServoParameters {
  ServoCommand command;
  float target_x;
  float error_x;
};

struct GimbalParameters {
  GimbalCommand command;
  float target_y;
  float error_y;
};

struct State {
  Confidence confidence;
  Lock lock;
  Trigger trigger;
};

struct ActuatorStatus {
  ActuatorState state;
  int count;
};

inline State decide(const TargetInput& target, const DecisionConfig& config, const ActuatorStatus& actuator_status)
{
  if (!target.visible) {
    return {Confidence::NONE, Lock::IDLE, Trigger::SKIP};
  }
  if (target.confidence < config.confidence_threshold) {
    return {Confidence::LOW, Lock::IDLE, Trigger::SKIP};
  }
  if (target.distance_m > config.max_distance_m) {
    return {Confidence::HIGH, Lock::TRACK, Trigger::SKIP};
  }
  if (actuator_status.state == ActuatorState::kReloading) {
    return {Confidence::HIGH, Lock::TRACK, Trigger::RELOADING};
  }
  return {Confidence::HIGH, Lock::TRACK, Trigger::REQUESTED};
}

inline ServoParameters calculate_servo_parameters(const ScreenResolutionConfig& resolution, float x_coord)
{
  float error_x = x_coord - resolution.width / 2;
  ServoCommand command{};

  if (error_x > 0) {
    command = ServoCommand::RIGHT;
  }
  else if (error_x < 0) {
    command = ServoCommand::LEFT;
  }
  else {
    command = ServoCommand::CENTER;
  }
  return ServoParameters{command, x_coord, error_x};
}

inline GimbalParameters calculate_gimbal_parameters(const ScreenResolutionConfig& resolution, float y_coord)
{
  float error_y = resolution.height / 2 - y_coord;
  GimbalCommand command{};

  if (error_y > 0) {
    command = GimbalCommand::UP;
  }
  else if (error_y < 0) {
    command = GimbalCommand::DOWN;
  }
  else {
    command = GimbalCommand::CENTER;
  }
  return GimbalParameters{command, y_coord, error_y};
}
}  // namespace antidrone_turret