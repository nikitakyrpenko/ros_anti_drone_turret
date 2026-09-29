#include "antidrone_turret/msg/servo_command.hpp"
#include "antidrone_turret/target_track_decisions.hpp"

#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>

namespace {
constexpr auto kServoTopic = "/servo/cmd";

using ServoType = antidrone_turret::msg::ServoCommand;
}  // namespace

class ServoDriverNode final : public rclcpp::Node {
public:
  ServoDriverNode()
    : rclcpp::Node("yaw_servo_driver_node")
  {
    subscription_ = this->create_subscription<ServoType>(kServoTopic, 10, [this](const ServoType& msg) { on_servo_message_received(msg); });
  }

private:
  auto on_servo_message_received(const ServoType& msg) -> void
  {
    const auto direction = static_cast<antidrone_turret::ServoCommand>(msg.direction);
    RCLCPP_INFO(
      get_logger(), "servo received dir=%s target_x=%.1f error_x=%.2f", antidrone_turret::to_string(direction), msg.target_x, msg.error_x);
  };

  rclcpp::Subscription<ServoType>::SharedPtr subscription_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ServoDriverNode>());
  rclcpp::shutdown();
  return 0;
}
