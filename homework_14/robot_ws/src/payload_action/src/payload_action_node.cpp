#include "rclcpp/rclcpp.hpp"
#include "underground_world/srv/payload_trigger.hpp"
#include "underground_world/msg/enemy_down.hpp"

using underground_world::msg::EnemyDown;
using underground_world::srv::PayloadTrigger;

class PayloadActionNode final : public rclcpp::Node {
public:
  PayloadActionNode()
    : Node("payload_action")
  {
    const auto qos = rclcpp::QoS{10};

    enemy_down_pub_ = this->create_publisher<EnemyDown>("/payload/enemy_down", qos);

    trigger_srv_ = this->create_service<PayloadTrigger>(
      "/payload/trigger", std::bind(&PayloadActionNode::on_trigger, this, std::placeholders::_1, std::placeholders::_2));
  }

private:
  void on_trigger(const PayloadTrigger::Request::SharedPtr request, PayloadTrigger::Response::SharedPtr response)
  {
    EnemyDown down;

    down.contact_id = request->contact_id;
    down.x = request->x;
    down.y = request->y;

    enemy_down_pub_->publish(down);
    response->accepted = true;
    response->reason = "payload delivered successfully";
    RCLCPP_INFO(get_logger(), "trigger contact_id=%d at (%d,%d)", request->contact_id, request->x, request->y);
  }

  // REGION C — declare the two members you used in Region A.
  rclcpp::Publisher<EnemyDown>::SharedPtr enemy_down_pub_;
  rclcpp::Service<PayloadTrigger>::SharedPtr trigger_srv_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PayloadActionNode>());
  rclcpp::shutdown();
  return 0;
}