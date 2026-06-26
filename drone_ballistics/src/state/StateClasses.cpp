#include "state/StateClasses.h"
#include <memory>
#include "Types.h"
#include "interfaces/IDroneState.h"

std::unique_ptr<IDroneState> StateStopped::execute(MissionContext& ctx)
{
  bool needBigTurn = std::fabs(ctx.angleDiff) > ctx.turnThreshold;
  if (needBigTurn) {
    float seed = std::fabs(ctx.angleDiff) / ctx.angularSpeed;
    ctx.command = {MotionKind::Accelerate, ctx.angleDiff, -seed};
    return std::make_unique<StateTurning>();
  }
  else {
    ctx.command = {MotionKind::Accelerate, ctx.angleDiff, -1.0f};
    return std::make_unique<StateAccelerating>();
  }
  return nullptr;
}
DronePhase StateStopped::name() const
{
  return STOPPED;
}

std::unique_ptr<IDroneState> StateAccelerating::execute(MissionContext& ctx)
{
  bool needBigTurn = fabs(ctx.angleDiff) > ctx.turnThreshold;
  if (needBigTurn) {
    ctx.command = {MotionKind::Decelerate, ctx.angleDiff, -1.0f};
    return std::make_unique<StateDecelerating>();
  }
  else if (ctx.currentSpeed >= ctx.attackSpeed) {
    ctx.currentSpeed = ctx.attackSpeed;  // Ensure we don't exceed max speed
    ctx.command = {MotionKind::Moving, ctx.angleDiff, -1.0f};
    return std::make_unique<StateMoving>();
  }
  else {
    ctx.command = {MotionKind::Accelerate, ctx.angleDiff, -1.0f};
    return std::make_unique<StateAccelerating>();
  }
}
DronePhase StateAccelerating::name() const
{
  return ACCELERATING;
}

std::unique_ptr<IDroneState> StateMoving::execute(MissionContext& ctx)
{
  bool needBigTurn = fabs(ctx.angleDiff) > ctx.turnThreshold;
  if (needBigTurn) {
    ctx.command = {MotionKind::Decelerate, ctx.angleDiff, -1.0f};
    return std::make_unique<StateDecelerating>();
  }
  ctx.command = {MotionKind::Moving, ctx.angleDiff, -1.0f};

  Coord bombLand{.x = 0.0, .y = 0.0};
  bombLand.x = ctx.pos.x + ctx.h_ammo * cos(ctx.direction);
  bombLand.y = ctx.pos.y + ctx.h_ammo * sin(ctx.direction);

  return std::make_unique<StateMoving>();
}
DronePhase StateMoving::name() const
{
  return MOVING;
}

std::unique_ptr<IDroneState> StateDecelerating::execute(MissionContext& ctx)
{
  bool needBigTurn = fabs(ctx.angleDiff) > ctx.turnThreshold;
  if (ctx.currentSpeed <= 0.0f) {
    ctx.command = {MotionKind::None, ctx.angleDiff, -1.0f};
    return std::make_unique<StateStopped>();
  }
  else if (!needBigTurn) {
    ctx.command = {MotionKind::Accelerate, ctx.angleDiff, -1.0f};
    return std::make_unique<StateAccelerating>();
  }
  else {
    ctx.command = {MotionKind::Decelerate, ctx.angleDiff, -1.0f};
    return std::make_unique<StateDecelerating>();
  }
}
DronePhase StateDecelerating::name() const
{
  return DECELERATING;
}

std::unique_ptr<IDroneState> StateTurning::execute(MissionContext& ctx)
{
  if (std::fabs(ctx.angleDiff) < ctx.turnThreshold) {
    ctx.command = {MotionKind::Accelerate, ctx.angleDiff, 0.0f};  // seed 0 = clear turn time
    return std::make_unique<StateAccelerating>();
  }
  else if (ctx.remainingTurnTime <= 0.0f) {
    float seed = std::fabs(ctx.angleDiff) / ctx.angularSpeed;
    ctx.command = {MotionKind::TurnInPlace, ctx.angleDiff, seed};
    return std::make_unique<StateTurning>();
  }
  ctx.command = {MotionKind::TurnInPlace, ctx.angleDiff, -1.0f};
  return std::make_unique<StateTurning>();
}
DronePhase StateTurning::name() const
{
  return TURNING;
}
