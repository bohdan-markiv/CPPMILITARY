#include "state/StateClasses.h"
#include <memory>
#include "Types.h"
#include "interfaces/IDroneState.h"

std::unique_ptr<IDroneState> StateStopped::execute(MissionContext& ctx)
{
  bool needBigTurn = std::fabs(ctx.angleDiff) > ctx.turnThreshold;
  if (needBigTurn) {
    float seed = std::fabs(ctx.angleDiff) / ctx.angularSpeed;
    ctx.command = {MotionKind::TurnInPlace, ctx.angleDiff, ctx.desiredDir, seed};
    return std::make_unique<StateTurning>();
  }
  ctx.command = {MotionKind::Accelerate, ctx.angleDiff, ctx.desiredDir, -1.0f};
  return std::make_unique<StateAccelerating>();
}
DronePhase StateStopped::name() const
{
  return STOPPED;
}

std::unique_ptr<IDroneState> StateAccelerating::execute(MissionContext& ctx)
{
  bool needBigTurn = std::fabs(ctx.angleDiff) > ctx.turnThreshold;
  if (needBigTurn) {
    ctx.command = {MotionKind::Decelerate, ctx.angleDiff, ctx.desiredDir, -1.0f};
    return std::make_unique<StateDecelerating>();
  }
  else if (ctx.currentSpeed >= ctx.attackSpeed) {
    ctx.command = {MotionKind::Moving, ctx.angleDiff, ctx.desiredDir, -1.0f};
    return std::make_unique<StateMoving>();
  }
  ctx.command = {MotionKind::Accelerate, ctx.angleDiff, ctx.desiredDir, -1.0f};
  return std::make_unique<StateAccelerating>();
}
DronePhase StateAccelerating::name() const
{
  return ACCELERATING;
}

std::unique_ptr<IDroneState> StateMoving::execute(MissionContext& ctx)
{
  bool needBigTurn = std::fabs(ctx.angleDiff) > ctx.turnThreshold;
  if (needBigTurn) {
    ctx.command = {MotionKind::Decelerate, ctx.angleDiff, ctx.desiredDir, -1.0f};
    return std::make_unique<StateDecelerating>();
  }
  ctx.command = {MotionKind::Moving, ctx.angleDiff, ctx.desiredDir, -1.0f};
  return std::make_unique<StateMoving>();
}
DronePhase StateMoving::name() const
{
  return MOVING;
}

std::unique_ptr<IDroneState> StateDecelerating::execute(MissionContext& ctx)
{
  bool needBigTurn = std::fabs(ctx.angleDiff) > ctx.turnThreshold;
  if (ctx.currentSpeed <= 0.0f) {
    ctx.command = {MotionKind::None, ctx.angleDiff, ctx.desiredDir, -1.0f};
    return std::make_unique<StateStopped>();
  }
  else if (!needBigTurn) {
    ctx.command = {MotionKind::Accelerate, ctx.angleDiff, ctx.desiredDir, -1.0f};
    return std::make_unique<StateAccelerating>();
  }
  ctx.command = {MotionKind::Decelerate, ctx.angleDiff, ctx.desiredDir, -1.0f};
  return std::make_unique<StateDecelerating>();
}
DronePhase StateDecelerating::name() const
{
  return DECELERATING;
}

std::unique_ptr<IDroneState> StateTurning::execute(MissionContext& ctx)
{
  if (std::fabs(ctx.angleDiff) < ctx.turnThreshold) {
    ctx.command = {MotionKind::Accelerate, ctx.angleDiff, ctx.desiredDir, 0.0f};
    return std::make_unique<StateAccelerating>();
  }
  else if (ctx.remainingTurnTime <= 0.0f) {
    float seed = std::fabs(ctx.angleDiff) / ctx.angularSpeed;
    ctx.command = {MotionKind::TurnInPlace, ctx.angleDiff, ctx.desiredDir, seed};
    return std::make_unique<StateTurning>();
  }
  ctx.command = {MotionKind::TurnInPlace, ctx.angleDiff, ctx.desiredDir, -1.0f};
  return std::make_unique<StateTurning>();
}
DronePhase StateTurning::name() const
{
  return TURNING;
}
