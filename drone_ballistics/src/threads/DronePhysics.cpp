#include "threads/DronePhysics.h"
#include <cmath>

void DronePhysics::init(Coord startPos, float initialDir, float attackSpeed, float accel, float angularSpeed, float physicsTimeStep)
{
  pos_ = startPos;
  direction_ = initialDir;
  currentSpeed_ = 0.0f;
  remainingTurnTime_ = 0.0f;
  timeSec_ = 0.0f;
  attackSpeed_ = attackSpeed;
  a_ = accel;
  angularSpeed_ = angularSpeed;
  dt_ = physicsTimeStep;
}
float DronePhysics::angleDifference(float from, float to)
{
  const float pi = std::acos(-1.0f);
  float diff = std::fmod(to - from + pi, 2.0f * pi);
  if (diff < 0)
    diff += 2.0f * pi;
  return diff - pi;
}
void DronePhysics::applyCruise()
{
  pos_.x += currentSpeed_ * std::cos(direction_) * dt_;
  pos_.y += currentSpeed_ * std::sin(direction_) * dt_;
}

void DronePhysics::applyTurnInPlace()
{
  float angleDiff = angleDifference(direction_, targetDir_);
  float turnStep = angularSpeed_ * dt_;
  if (turnStep > std::fabs(angleDiff))
    turnStep = std::fabs(angleDiff);
  direction_ += (angleDiff > 0 ? 1.0f : -1.0f) * turnStep;
  remainingTurnTime_ -= dt_;
  if (remainingTurnTime_ < 0.0f)
    remainingTurnTime_ = 0.0f;
}

void DronePhysics::applyDeceleration(float angleDiff)
{
  (void)angleDiff;
  currentSpeed_ -= a_ * dt_;
  if (currentSpeed_ < 0.0f)
    currentSpeed_ = 0.0f;
  applyCruise();
}

void DronePhysics::applySmallTurn()
{
  float angleDiff = angleDifference(direction_, targetDir_);  // recompute each sub-step
  if (std::fabs(angleDiff) > 1e-4f) {
    float turnStep = angularSpeed_ * dt_;
    if (turnStep > std::fabs(angleDiff))
      turnStep = std::fabs(angleDiff);
    direction_ += (angleDiff > 0 ? 1.0f : -1.0f) * turnStep;
  }
}

void DronePhysics::applyMoving()
{
  applySmallTurn();
  applyCruise();
}

void DronePhysics::applyAcceleration()
{
  currentSpeed_ += a_ * dt_;
  if (currentSpeed_ > attackSpeed_)
    currentSpeed_ = attackSpeed_;
  applySmallTurn();
  applyCruise();
}

void DronePhysics::pushCommand(DroneCommand cmd)
{
  commandQueue_.push(cmd);
}

void DronePhysics::step(float dt)
{
  std::lock_guard<std::mutex> lock(mutex_);

  // Drain queue: latch the newest command (don't apply motion here)
  DroneCommand cmd;
  while (commandQueue_.try_pop(cmd)) {
    current_ = cmd;
    targetDir_ = cmd.targetDir;
    if (cmd.remainingTurnSeed >= 0.0f)
      remainingTurnTime_ = cmd.remainingTurnSeed;
  }

  // Apply the latched command EVERY step (runs all 10 sub-steps)
  switch (current_.motion) {
    case MotionKind::TurnInPlace:
      applyTurnInPlace();
      break;
    case MotionKind::Accelerate:
      applyAcceleration();
      break;
    case MotionKind::Moving:
      applyMoving();
      break;
    case MotionKind::Decelerate:
      applyDeceleration(current_.angleDiff);
      break;
    case MotionKind::None:
      break;
  }

  timeSec_ += dt;
}

DroneTelemetry DronePhysics::getTelemetry() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  DroneTelemetry t;
  t.pos = pos_;
  t.direction = direction_;
  t.currentSpeed = currentSpeed_;
  t.speed.x = currentSpeed_ * std::cos(direction_);
  t.speed.y = currentSpeed_ * std::sin(direction_);
  t.remainingTurnTime = remainingTurnTime_;
  t.timeSecSinceStart = timeSec_;
  return t;
}