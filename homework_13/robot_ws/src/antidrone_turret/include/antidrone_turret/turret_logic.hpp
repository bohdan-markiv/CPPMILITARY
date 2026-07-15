#ifndef ANTIDRONE_TURRET_TURRET_LOGIC_HPP
#define ANTIDRONE_TURRET_TURRET_LOGIC_HPP

namespace antidrone_turret::logic {

// --- Plain enums that mirror the .msg constants (but are NOT ROS types) ---

enum class TargetState { kNone, kLowConfidence, kLocked };
enum class ActionState { kIdle, kTrack };
enum class TriggerState { kSkip, kRequested, kReloading };

// Direction is shared shape for both servo (LEFT/CENTER/RIGHT)
// and gimbal (DOWN/CENTER/UP). We use -1/0/1 to match the .msg.
enum class Direction { kNegative = -1, kCenter = 0, kPositive = 1 };

// The actuator's last known state, as seen by the controller.
enum class ActuatorState { kReady, kReloading };

// --- Plain result structs (mirror the .msg fields, plain C++) ---

struct ServoResult {
  Direction direction;
  float target_x;
  float error_x;
};

struct GimbalResult {
  Direction direction;
  float target_y;
  float error_y;
};

struct TurretStatusResult {
  TargetState target_state;
  ActionState action;
  TriggerState trigger_state;
  float confidence;
  float distance_m;
};

// --- The five pure functions ---

// 1. Target evaluation.
//    not visible                    -> kNone
//    visible but conf < threshold   -> kLowConfidence
//    visible and conf >= threshold  -> kLocked
inline TargetState evaluateTarget(bool visible, float confidence, float confidence_threshold)
{
  if (!visible) {
    return TargetState::kNone;
  }
  else if (confidence < confidence_threshold) {
    return TargetState::kLowConfidence;
  }
  else {
    return TargetState::kLocked;
  }
}

// 2. Servo (horizontal) command from the target's x.
//    error_x = x - 320
//    error_x > 0 -> kPositive (RIGHT), < 0 -> kNegative (LEFT), == 0 -> kCenter
inline ServoResult servoCommand(float x)
{
  // TODO (blank 2): compute error_x, pick direction, fill the struct.
  float error_x = x - 320.0f;
  Direction direction;
  if (error_x > 0) {
    direction = Direction::kPositive;
  }
  else if (error_x < 0) {
    direction = Direction::kNegative;
  }
  else {
    direction = Direction::kCenter;
  }
  return ServoResult{direction, x, error_x};
}

// 3. Gimbal (vertical) command from the target's y.
//    error_y = 240 - y   (note: inverted, because screen y grows downward)
//    error_y > 0 -> kPositive (UP), < 0 -> kNegative (DOWN), == 0 -> kCenter
inline GimbalResult gimbalCommand(float y)
{
  // TODO (blank 3): compute error_y, pick direction, fill the struct.
  float error_y = 240.0f - y;
  Direction direction;
  if (error_y > 0) {
    direction = Direction::kPositive;
  }
  else if (error_y < 0) {
    direction = Direction::kNegative;
  }
  else {
    direction = Direction::kCenter;
  }
  return GimbalResult{direction, y, error_y};
}

// 4. Trigger decision. Only meaningful for a LOCKED target; the caller
//    only calls this when the target is good. Rules:
//    distance_m > max_distance_m           -> kSkip
//    close AND actuator kReady             -> kRequested
//    close AND actuator kReloading         -> kReloading
inline TriggerState decideTrigger(float distance_m, float max_distance_m, ActuatorState actuator_state)
{
  // TODO (blank 4): distance check first, then branch on actuator_state.
  if (distance_m > max_distance_m) {
    return TriggerState::kSkip;
  }
  if (actuator_state == ActuatorState::kReady) {
    return TriggerState::kRequested;
  }
  return TriggerState::kReloading;
}

// 5. Assemble the full status. This ties the pieces together for a target.
//    Given the evaluated target_state and the trigger decision, set action:
//      LOCKED -> kTrack, anything else -> kIdle
//    Copy confidence and distance_m straight through.
inline TurretStatusResult buildTurretStatus(TargetState target_state, TriggerState trigger_state, float confidence, float distance_m)
{
  // TODO (blank 5): set action from target_state, fill the struct
  if (target_state == TargetState::kLocked) {
    return TurretStatusResult{target_state, ActionState::kTrack, trigger_state, confidence, distance_m};
  }
  else {
    return TurretStatusResult{target_state, ActionState::kIdle, trigger_state, confidence, distance_m};
  }
}

}  // namespace antidrone_turret::logic

#endif  // ANTIDRONE_TURRET_TURRET_LOGIC_HPP