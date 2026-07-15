#include <memory>
#include <cstdint>
#include <rclcpp/rclcpp.hpp>

#include "antidrone_turret/turret_logic.hpp"

#include "antidrone_turret/msg/target.hpp"
#include "antidrone_turret/msg/actuator_status.hpp"
#include "antidrone_turret/msg/gimbal_command.hpp"
#include "antidrone_turret/msg/servo_command.hpp"
#include "antidrone_turret/msg/turret_status.hpp"
#include "antidrone_turret/srv/trigger_actuator.hpp"

namespace {
constexpr auto kTargetTopic = "/perception/target";
constexpr auto kActuatorStatusTopic = "/actuator/status";
constexpr auto kGimbalTopic = "/gimbal/cmd";
constexpr auto kServoTopic = "/servo/cmd";
constexpr auto kTurretStatusTopic = "/turret/status";
constexpr auto kTriggerService = "/actuator/trigger";

std::int8_t to_int8(antidrone_turret::logic::Direction d)
{
  return static_cast<std::int8_t>(d);
}
}  // namespace

class TurretControllerNode final : public rclcpp::Node {
public:
  using Target = antidrone_turret::msg::Target;
  using ActuatorStatus = antidrone_turret::msg::ActuatorStatus;
  using GimbalCommand = antidrone_turret::msg::GimbalCommand;
  using ServoCommand = antidrone_turret::msg::ServoCommand;
  using TurretStatus = antidrone_turret::msg::TurretStatus;
  using TriggerActuator = antidrone_turret::srv::TriggerActuator;

  TurretControllerNode()
    : Node("turret_controller_node")
  {
    confidence_threshold_ = declare_parameter<double>("confidence_threshold", 0.80);
    max_distance_m_ = declare_parameter<double>("max_distance_m", 30.0);

    gimbal_pub_ = create_publisher<GimbalCommand>(kGimbalTopic, 10);
    servo_pub_ = create_publisher<ServoCommand>(kServoTopic, 10);
    status_pub_ = create_publisher<TurretStatus>(kTurretStatusTopic, 10);

    trigger_client_ = create_client<TriggerActuator>(kTriggerService);

    actuator_sub_ = create_subscription<ActuatorStatus>(
      kActuatorStatusTopic, 10, [this](const ActuatorStatus::SharedPtr msg) { on_actuator_status(msg); });

    target_sub_ = create_subscription<Target>(kTargetTopic, 10, [this](const Target::SharedPtr msg) { on_target(msg); });

    RCLCPP_INFO(get_logger(), "turret_controller_node ready");
  }

private:
  // Store the latest actuator state so the target callback can read it.
  void on_actuator_status(const ActuatorStatus::SharedPtr msg) { last_actuator_ready_ = (msg->state == ActuatorStatus::READY); }

  void on_target(const Target::SharedPtr msg)
  {
    using namespace antidrone_turret::logic;

    const auto threshold = static_cast<float>(confidence_threshold_);
    const auto max_dist = static_cast<float>(max_distance_m_);

    TargetState target_state = evaluateTarget(msg->visible, msg->confidence, threshold);

    // We'll fill trigger_state during the locked branch; default it to skip.
    TriggerState trigger_state = TriggerState::kSkip;

    if (target_state == TargetState::kLocked) {
      // Build and publish the servo command from msg->x
      const auto servo = servoCommand(msg->x);
      ServoCommand servo_msg;
      servo_msg.direction = to_int8(servo.direction);
      servo_msg.target_x = servo.target_x;
      servo_msg.error_x = servo.error_x;
      servo_pub_->publish(servo_msg);

      const auto gimbal = gimbalCommand(msg->y);
      GimbalCommand gimbal_msg;
      gimbal_msg.direction = to_int8(gimbal.direction);
      gimbal_msg.target_y = gimbal.target_y;
      gimbal_msg.error_y = gimbal.error_y;
      gimbal_pub_->publish(gimbal_msg);
      trigger_state = decideTrigger(msg->distance_m, max_dist, last_actuator_ready_ ? ActuatorState::kReady : ActuatorState::kReloading);

      if (trigger_state == TriggerState::kRequested) {
        auto request = std::make_shared<TriggerActuator::Request>();
        request->confidence = msg->confidence;
        request->distance_m = msg->distance_m;
        trigger_client_->async_send_request(request);
      }
    }

    // --- always: build and publish TurretStatus ---
    const auto status = buildTurretStatus(target_state, trigger_state, msg->confidence, msg->distance_m);

    TurretStatus status_msg;
    status_msg.target_state = static_cast<std::uint8_t>(status.target_state);
    status_msg.action = static_cast<std::uint8_t>(status.action);
    status_msg.trigger_state = static_cast<std::uint8_t>(status.trigger_state);
    status_msg.confidence = status.confidence;
    status_msg.distance_m = status.distance_m;
    status_pub_->publish(status_msg);
  }

  double confidence_threshold_{0.80};
  double max_distance_m_{30.0};
  bool last_actuator_ready_{true};  // actuator starts READY

  rclcpp::Publisher<GimbalCommand>::SharedPtr gimbal_pub_;
  rclcpp::Publisher<ServoCommand>::SharedPtr servo_pub_;
  rclcpp::Publisher<TurretStatus>::SharedPtr status_pub_;
  rclcpp::Client<TriggerActuator>::SharedPtr trigger_client_;
  rclcpp::Subscription<ActuatorStatus>::SharedPtr actuator_sub_;
  rclcpp::Subscription<Target>::SharedPtr target_sub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TurretControllerNode>());
  rclcpp::shutdown();
  return 0;
}