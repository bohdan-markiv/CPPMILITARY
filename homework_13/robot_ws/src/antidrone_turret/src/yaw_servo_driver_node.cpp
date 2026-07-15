#include <memory>
#include <cstdint>
#include <rclcpp/rclcpp.hpp>

#include "antidrone_turret/msg/servo_command.hpp"

namespace {
constexpr auto kServoTopic = "/servo/cmd";

// Turn the int8 direction back into a readable name for the log.
const char* direction_name(std::int8_t direction)
{
  using ServoCommand = antidrone_turret::msg::ServoCommand;

  if (direction == ServoCommand::LEFT) {
    return "LEFT";
  }
  else if (direction == ServoCommand::RIGHT) {
    return "RIGHT";
  }
  else if (direction == ServoCommand::CENTER) {
    return "CENTER";
  }
  else {
    return "UNKNOWN";
  }
}
}  // namespace

class ServoDriverNode final : public rclcpp::Node {
public:
  using ServoCommand = antidrone_turret::msg::ServoCommand;

  ServoDriverNode()
    : Node("yaw_servo_driver_node")
  {
    subscription_ = create_subscription<ServoCommand>(kServoTopic, 10, [this](const ServoCommand::SharedPtr msg) { on_command(msg); });

    RCLCPP_INFO(get_logger(), "yaw_servo_driver_node listening on %s", kServoTopic);
  }

private:
  void on_command(const ServoCommand::SharedPtr msg)
  {
    RCLCPP_INFO(
      get_logger(), "отримав: direction=%s target_x=%.1f error_x=%.1f", direction_name(msg->direction), msg->target_x, msg->error_x);
  }

  rclcpp::Subscription<ServoCommand>::SharedPtr subscription_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ServoDriverNode>());
  rclcpp::shutdown();
  return 0;
}