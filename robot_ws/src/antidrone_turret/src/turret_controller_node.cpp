#include "antidrone_turret/msg/actuator_status.hpp"
#include "antidrone_turret/msg/gimbal_command.hpp"
#include "antidrone_turret/msg/target.hpp"
#include "antidrone_turret/msg/servo_command.hpp"
#include "antidrone_turret/srv/trigger_actuator.hpp"
#include "antidrone_turret/target_track_decisions.hpp"
#include "antidrone_turret/msg/turret_status.hpp"

#include <cstdint>
#include <memory>
#include <rclcpp/client.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>

namespace {
constexpr auto kTargetTopic = "/perception/target";
constexpr auto kGimbalTopic = "/gimbal/cmd";
constexpr auto kServoTopic = "/servo/cmd";
constexpr auto kTurretStatusTopic = "/turret/status";
constexpr auto kActuatorStatusTopic = "/actuator/status";
constexpr auto kTriggerService = "/actuator/trigger";

using ServoType = antidrone_turret::msg::ServoCommand;
using GimbalType = antidrone_turret::msg::GimbalCommand;
using TargetType = antidrone_turret::msg::Target;
using TurretStatusType = antidrone_turret::msg::TurretStatus;
using ActuatorStatusType = antidrone_turret::msg::ActuatorStatus;
using TriggerActuatorType = antidrone_turret::srv::TriggerActuator;

ServoType map(const antidrone_turret::ServoParameters& servo_parameters)
{
  return ServoType()
    .set__direction(static_cast<int8_t>(servo_parameters.command))
    .set__target_x(servo_parameters.target_x)
    .set__error_x(servo_parameters.error_x);
}

GimbalType map(const antidrone_turret::GimbalParameters& gimbal_parameters)
{
  return GimbalType()
    .set__direction(static_cast<int8_t>(gimbal_parameters.command))
    .set__target_y(gimbal_parameters.target_y)
    .set__error_y(gimbal_parameters.error_y);
}

TurretStatusType map(const antidrone_turret::State& state, const antidrone_turret::TargetInput target)
{
  return TurretStatusType()
    .set__target_state(static_cast<int8_t>(state.confidence))
    .set__action(static_cast<int8_t>(state.lock))
    .set__trigger_state(static_cast<int8_t>(state.trigger))
    .set__confidence(target.confidence)
    .set__distance_m(target.distance_m);
}

antidrone_turret::TargetInput map(const TargetType& tgt)
{
  return {.visible = tgt.visible, .x = tgt.x, .y = tgt.y, .distance_m = tgt.distance_m, .confidence = tgt.confidence};
}

antidrone_turret::ActuatorStatus map(const ActuatorStatusType& act)
{
  return {.state = static_cast<antidrone_turret::ActuatorState>(act.state), .count = static_cast<int>(act.trigger_count)};
}
}  // namespace

class TurretControllerNode final : public rclcpp::Node {
public:
  TurretControllerNode()
    : rclcpp::Node("turret_controller_node")
    , gimbal_publisher_{create_publisher<GimbalType>(kGimbalTopic, 10)}
    , servo_publisher_{create_publisher<ServoType>(kServoTopic, 10)}
    , turret_status_publisher_{create_publisher<TurretStatusType>(kTurretStatusTopic, 10)}
    , actuator_client_{create_client<TriggerActuatorType>(kTriggerService)}
    , target_subscription_{create_subscription<TargetType>(
        kTargetTopic, 10, [this](const TargetType& msg_tgt) { on_target_recieved(msg_tgt); })}
    , actuator_subscription_{create_subscription<ActuatorStatusType>(
        kActuatorStatusTopic, 10, [this](const ActuatorStatusType& msg_act) { on_actuator_status_recieved(msg_act); })}
  {
    const auto confidence_threshold = declare_parameter<double>("confidence_threshold", 0.8);
    const auto max_distance_m = declare_parameter<double>("max_distance_m", 30.0F);

    const auto screen_width = declare_parameter<double>("screen_width", 640.0);
    const auto screen_height = declare_parameter<double>("screen_height", 480.0);

    decision_config_ = {static_cast<float>(confidence_threshold), static_cast<float>(max_distance_m)};
    screen_resolution_config_ = {static_cast<float>(screen_width), static_cast<float>(screen_height)};
  }

private:
  auto on_target_recieved(const TargetType& msg_tgt) -> void
  {
    const auto target = map(msg_tgt);

    RCLCPP_INFO(get_logger(),
                "target received visible=%s x=%.1f y=%.1f distance_m=%.1f confidence=%.2f",
                target.visible ? "true" : "false",
                target.x,
                target.y,
                target.distance_m,
                target.confidence);

    const auto state = antidrone_turret::decide(target, decision_config_, current_status_);

    RCLCPP_INFO(get_logger(),
                "decision confidence=%s lock=%s trigger=%s",
                antidrone_turret::to_string(state.confidence),
                antidrone_turret::to_string(state.lock),
                antidrone_turret::to_string(state.trigger));

    if (state.lock == antidrone_turret::Lock::TRACK) {
      const auto servo = antidrone_turret::calculate_servo_parameters(screen_resolution_config_, msg_tgt.x);
      const auto gimbal = antidrone_turret::calculate_gimbal_parameters(screen_resolution_config_, msg_tgt.y);

      servo_publisher_->publish(map(servo));
      gimbal_publisher_->publish(map(gimbal));
    }

    if (state.trigger == antidrone_turret::Trigger::REQUESTED) {
      auto actuator_request = std::make_shared<TriggerActuatorType::Request>();
      actuator_request->confidence = target.confidence;
      actuator_request->distance_m = target.distance_m;

      this->actuator_client_->async_send_request(
        actuator_request,
        [this](rclcpp::Client<TriggerActuatorType>::SharedFuture
                 future) {  // NOLINT(performance-unnecessary-value-param) rclcpp callback signature requires by-value
          on_turret_actuator_responded(future);
        });
    }
    turret_status_publisher_->publish(map(state, target));
  }

  auto on_actuator_status_recieved(const ActuatorStatusType& msg_act) -> void
  {
    const auto previous_state = current_status_.state;
    current_status_ = map(msg_act);

    if (current_status_.state != previous_state) {
      RCLCPP_INFO(get_logger(),
                  "actuator status state=%s trigger_count=%d",
                  antidrone_turret::to_string(current_status_.state),
                  current_status_.count);
    }
  }

  auto on_turret_actuator_responded(const rclcpp::Client<TriggerActuatorType>::SharedFuture& future) -> void
  {
    const auto& response = future.get();
    RCLCPP_INFO(get_logger(),
                "actuator trigger response accepted=%s trigger_count=%u",
                response->accepted ? "true" : "false",
                response->trigger_count);
  }

  antidrone_turret::ScreenResolutionConfig screen_resolution_config_;
  antidrone_turret::DecisionConfig decision_config_;

  antidrone_turret::ActuatorStatus current_status_{};

  rclcpp::Publisher<GimbalType>::SharedPtr gimbal_publisher_;
  rclcpp::Publisher<ServoType>::SharedPtr servo_publisher_;
  rclcpp::Publisher<TurretStatusType>::SharedPtr turret_status_publisher_;
  rclcpp::Client<TriggerActuatorType>::SharedPtr actuator_client_;
  rclcpp::Subscription<TargetType>::SharedPtr target_subscription_;
  rclcpp::Subscription<ActuatorStatusType>::SharedPtr actuator_subscription_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TurretControllerNode>());
  rclcpp::shutdown();
  return 0;
}