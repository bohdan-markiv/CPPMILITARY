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

void DronePhysics::applyCruise()
{
  pos_.x += currentSpeed_ * std::cos(direction_) * dt_;
  pos_.y += currentSpeed_ * std::sin(direction_) * dt_;
}

void DronePhysics::applyTurnInPlace(float angleDiff)
{
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

void DronePhysics::applySmallTurn(float angleDiff)
{
  if (std::fabs(angleDiff) > 1e-4f) {
    float turnStep = angularSpeed_ * dt_;
    if (turnStep > std::fabs(angleDiff))
      turnStep = std::fabs(angleDiff);
    direction_ += (angleDiff > 0 ? 1.0f : -1.0f) * turnStep;
  }
}

void DronePhysics::applyMoving(float angleDiff)
{
  applySmallTurn(angleDiff);
  applyCruise();
}

void DronePhysics::applyAcceleration(float angleDiff)
{
  currentSpeed_ += a_ * dt_;
  if (currentSpeed_ > attackSpeed_)
    currentSpeed_ = attackSpeed_;
  applySmallTurn(angleDiff);
  applyCruise();
}

void DronePhysics::pushCommand(DroneCommand cmd)
{
  commandQueue_.push(cmd);
}

void DronePhysics::step(float dt)
{
  std::lock_guard<std::mutex> lock(mutex_);

  DroneCommand cmd;
  // Drain all pending commands; the last one wins as current motion intent.
  // In Phase 1 (manual stepping) there's exactly one per call.
  while (commandQueue_.try_pop(cmd)) {
    if (cmd.remainingTurnSeed >= 0.0f)
      remainingTurnTime_ = cmd.remainingTurnSeed;

    switch (cmd.motion) {
      case MotionKind::TurnInPlace:
        applyTurnInPlace(cmd.angleDiff);
        break;
      case MotionKind::Accelerate:
        applyAcceleration(cmd.angleDiff);
        break;
      case MotionKind::Moving:
        applyMoving(cmd.angleDiff);
        break;
      case MotionKind::Decelerate:
        applyDeceleration(cmd.angleDiff);
        break;
      case MotionKind::None:
        break;
    }
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