#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "antidrone_turret/msg/gimbal_command.hpp"

namespace {
constexpr auto kGimbalTopic = "/gimbal/cmd";

// Turn the int8 direction back into a readable name for the log.
const char* direction_name(std::int8_t direction)
{
  using GimbalCommand = antidrone_turret::msg::GimbalCommand;

  if (direction == GimbalCommand::UP) {
    return "UP";
  }
  else if (direction == GimbalCommand::DOWN) {
    return "DOWN";
  }
  else if (direction == GimbalCommand::CENTER) {
    return "CENTER";
  }
  else {
    return "UNKNOWN";
  }
}
}  // namespace

class GimbalDriverNode final : public rclcpp::Node {
public:
  using GimbalCommand = antidrone_turret::msg::GimbalCommand;

  GimbalDriverNode()
    : Node("gimbal_driver_node")
  {
    subscription_ = create_subscription<GimbalCommand>(kGimbalTopic, 10, [this](const GimbalCommand::SharedPtr msg) { on_command(msg); });

    RCLCPP_INFO(get_logger(), "gimbal_driver_node listening on %s", kGimbalTopic);
  }

private:
  void on_command(const GimbalCommand::SharedPtr msg)
  {
    RCLCPP_INFO(
      get_logger(), "отримав: direction=%s target_y=%.1f error_y=%.1f", direction_name(msg->direction), msg->target_y, msg->error_y);
  }

  rclcpp::Subscription<GimbalCommand>::SharedPtr subscription_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GimbalDriverNode>());
  rclcpp::shutdown();
  return 0;
}