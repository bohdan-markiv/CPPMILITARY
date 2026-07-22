#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <chrono>
#include "rclcpp/rclcpp.hpp"

#include "underground_world/msg/local_scan.hpp"
#include "underground_world/msg/move_command.hpp"
#include "underground_world/msg/student_status.hpp"
#include "underground_world/srv/payload_trigger.hpp"

#include "mission_explorer/map_memory.hpp"

using underground_world::msg::LocalScan;
using underground_world::msg::MoveCommand;
using underground_world::msg::StudentStatus;
using underground_world::srv::PayloadTrigger;

namespace {

explorer::CellKind kind_from_string(const std::string& s)
{
  if (s == "#")
    return explorer::CellKind::Wall;
  if (s == "C")
    return explorer::CellKind::Contact;
  if (s == "." || s == "S" || s == "x")
    return explorer::CellKind::Floor;
  return explorer::CellKind::Wall;
}

// Given current (cx,cy) and an adjacent target (tx,ty), return the
// MoveCommand direction constant. Assumes the target IS adjacent.
uint8_t direction_to(int cx, int cy, int tx, int ty)
{
  if (ty == cy - 1)
    return MoveCommand::UP;  // y decreases → UP
  if (ty == cy + 1)
    return MoveCommand::DOWN;  // y increases → DOWN
  if (tx == cx - 1)
    return MoveCommand::LEFT;
  if (tx == cx + 1)
    return MoveCommand::RIGHT;
  return MoveCommand::UP;  // unreachable if target is truly adjacent
}

}  // namespace

class MissionExplorerNode final : public rclcpp::Node {
public:
  MissionExplorerNode()
    : Node("mission_explorer")
  {
    const auto qos = rclcpp::QoS{10};

    cmd_move_pub_ = create_publisher<MoveCommand>("/robot/cmd_move", qos);
    status_pub_ = create_publisher<StudentStatus>("/student/status", qos);

    trigger_client_ = create_client<PayloadTrigger>("/payload/trigger");

    scan_sub_ = create_subscription<LocalScan>("/robot/local_scan", qos, [this](const LocalScan::SharedPtr msg) { on_scan(*msg); });
    kickstart_timer_ = create_wall_timer(std::chrono::seconds(1), [this]() {
      if (first_scan_received_) {
        kickstart_timer_->cancel();
        return;
      }
      RCLCPP_WARN(get_logger(), "no scan after 1s — sending kickstart");
      MoveCommand kick;
      kick.direction = 99;  // out-of-range → forces republish, no invalid_move
      cmd_move_pub_->publish(kick);
    });

    RCLCPP_INFO(get_logger(), "mission_explorer ready");
  }

private:
  void publish_status(uint8_t state)
  {
    StudentStatus msg;
    msg.state = state;
    status_pub_->publish(msg);
  }

  void trigger_contact(int contact_id, int x, int y)
  {
    auto request = std::make_shared<PayloadTrigger::Request>();
    request->contact_id = contact_id;
    request->x = x;
    request->y = y;

    trigger_client_->async_send_request(request, [this](rclcpp::Client<PayloadTrigger>::SharedFuture future) {
      const auto response = future.get();
      RCLCPP_INFO(get_logger(), "trigger accepted=%s reason=%s", response->accepted ? "true" : "false", response->reason.c_str());
    });
  }

  // Decide and send exactly ONE move for the current robot position.
  // Returns true if a move was sent, false if exploration is done.
  bool decide_and_move(const explorer::Coord& robot)
  {
    // Ensure the robot's current cell is on the stack (first scan seeds it).
    if (path_stack_.empty() || path_stack_.back() != robot) {
      path_stack_.push_back(robot);
    }

    const int cx = robot.first, cy = robot.second;

    // The four neighbours, in a fixed order.
    const explorer::Coord neighbours[4] = {{cx, cy - 1}, {cx, cy + 1}, {cx - 1, cy}, {cx + 1, cy}};

    // Dive: first walkable, unvisited neighbour wins.
    for (const auto& nb : neighbours) {
      if (map_.is_walkable(nb) && !map_.is_visited(nb)) {
        MoveCommand cmd;
        cmd.direction = direction_to(cx, cy, nb.first, nb.second);
        cmd_move_pub_->publish(cmd);
        return true;
      }
    }

    // No unvisited neighbour → backtrack.
    path_stack_.pop_back();  // leave the dead-end cell
    if (path_stack_.empty()) {
      publish_status(StudentStatus::DONE);
      RCLCPP_INFO(get_logger(), "exploration DONE");
      return false;
    }
    const auto& back = path_stack_.back();
    MoveCommand cmd;
    cmd.direction = direction_to(cx, cy, back.first, back.second);
    cmd_move_pub_->publish(cmd);
    return true;
  }

  void on_scan(const LocalScan& scan)
  {
    first_scan_received_ = true;
    const explorer::Coord robot{scan.robot_x, scan.robot_y};
    map_.mark_visited(robot);

    for (const auto& cell : scan.cells) {
      const explorer::Coord at{cell.x, cell.y};
      map_.observe(at, kind_from_string(cell.cell_type));
    }

    // Contact handling: fire a trigger for every visible C.
    bool engaging = false;
    for (const auto& cell : scan.cells) {
      if (cell.cell_type == "C") {
        RCLCPP_INFO(get_logger(), "contact visible id=%d at (%d,%d)", cell.contact_id, cell.x, cell.y);
        trigger_contact(cell.contact_id, cell.x, cell.y);
        engaging = true;
      }
    }

    if (engaging) {
      publish_status(StudentStatus::ENGAGING);
    }
    else {
      publish_status(StudentStatus::EXPLORING);
      decide_and_move(robot);
    }

    RCLCPP_INFO(get_logger(),
                "scan robot=(%d,%d) cells=%zu engaging=%s stack=%zu",
                scan.robot_x,
                scan.robot_y,
                scan.cells.size(),
                engaging ? "true" : "false",
                path_stack_.size());
  }

  rclcpp::Publisher<MoveCommand>::SharedPtr cmd_move_pub_;
  rclcpp::Publisher<StudentStatus>::SharedPtr status_pub_;
  rclcpp::Client<PayloadTrigger>::SharedPtr trigger_client_;
  rclcpp::Subscription<LocalScan>::SharedPtr scan_sub_;
  bool first_scan_received_ = false;
  rclcpp::TimerBase::SharedPtr kickstart_timer_;
  explorer::MapMemory map_;
  std::vector<explorer::Coord> path_stack_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MissionExplorerNode>());
  rclcpp::shutdown();
  return 0;
}