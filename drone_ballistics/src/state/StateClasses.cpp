#include "state/StateClasses.h"
#include <memory>
#include "Types.h"
#include "helpers.h"
#include "interfaces/IDroneState.h"
#include "iostream"

void applyCruise(MissionContext& ctx)
{
  ctx.pos.x += ctx.currentSpeed * cos(ctx.direction) * ctx.simTimeStep;
  ctx.pos.y += ctx.currentSpeed * sin(ctx.direction) * ctx.simTimeStep;
}
void applyTurnInPlace(MissionContext& ctx)
{
  double turnStep = ctx.angularSpeed * ctx.simTimeStep;
  if (turnStep > fabs(ctx.angleDiff)) {
    turnStep = fabs(ctx.angleDiff);
  }
  ctx.direction += (ctx.angleDiff > 0 ? 1.0 : -1.0) * turnStep;
  ctx.remainingTurnTime -= ctx.simTimeStep;
  if (ctx.remainingTurnTime < 0.0) {
    ctx.remainingTurnTime = 0.0;
  }
}

void applyDeceleration(MissionContext& ctx)
{
  ctx.currentSpeed -= ctx.a * ctx.simTimeStep;
  if (ctx.currentSpeed < 0.0) {
    ctx.currentSpeed = 0.0;
  }
  applyCruise(ctx);
}
void applySmallTurn(MissionContext& ctx)
{
  if (fabs(ctx.angleDiff) > 1e-4f) {
    double turnStep = ctx.angularSpeed * ctx.simTimeStep;
    if (turnStep > fabs(ctx.angleDiff)) {
      turnStep = fabs(ctx.angleDiff);
    }
    ctx.direction += (ctx.angleDiff > 0 ? 1.0 : -1.0) * turnStep;
  }
}
void applyMoving(MissionContext& ctx)
{
  applySmallTurn(ctx);
  applyCruise(ctx);
}

void applyAcceleration(MissionContext& ctx)
{
  ctx.currentSpeed += ctx.a * ctx.simTimeStep;
  if (ctx.currentSpeed > ctx.attackSpeed) {
    ctx.currentSpeed = ctx.attackSpeed;
  }
  applySmallTurn(ctx);
  applyCruise(ctx);
}
std::unique_ptr<IDroneState> StateStopped::execute(MissionContext& ctx)
{
  bool needBigTurn = fabs(ctx.angleDiff) > ctx.turnThreshold;
  if (needBigTurn) {
    ctx.remainingTurnTime = fabs(ctx.angleDiff) / ctx.angularSpeed;
    applyTurnInPlace(ctx);
    return std::make_unique<StateTurning>();
  }
  else {
    applyAcceleration(ctx);
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
    applyDeceleration(ctx);
    return std::make_unique<StateDecelerating>();
  }
  else if (ctx.currentSpeed >= ctx.attackSpeed) {
    ctx.currentSpeed = ctx.attackSpeed;  // Ensure we don't exceed max speed
    applyMoving(ctx);
    return std::make_unique<StateMoving>();
  }
  else {
    applyAcceleration(ctx);
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
    applyDeceleration(ctx);
    return std::make_unique<StateDecelerating>();
  }
  applyMoving(ctx);

  Coord bombLand{.x = 0.0, .y = 0.0};
  bombLand.x = ctx.pos.x + ctx.h_ammo * cos(ctx.direction);
  bombLand.y = ctx.pos.y + ctx.h_ammo * sin(ctx.direction);

  Coord hitCoord = ctx.predictedTarget;  // Assuming predictedTarget is the expected position of the target at impact time
  double missDistance = distanceCalculation(bombLand, hitCoord);
  if (missDistance <= ctx.hitRadius) {
    ctx.targetHit = true;
    std::cout << "Target " << ctx.targetIdx << " HIT at t=" << ctx.t << "s (step " << ctx.N << ")" << std::endl;
    std::cout << "  Drone:  (" << ctx.pos.x << ", " << ctx.pos.y << ")  dir=" << ctx.direction << std::endl;
    std::cout << "  Bomb lands at:   (" << bombLand.x << ", " << bombLand.y << ")" << std::endl;
    std::cout << "  Target at impact:(" << hitCoord.x << ", " << hitCoord.y << ")" << std::endl;
    std::cout << "  Miss distance:   " << missDistance << " m" << std::endl;
  }
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
    ctx.currentSpeed = 0.0f;
    return std::make_unique<StateStopped>();
  }
  else if (!needBigTurn) {
    applyAcceleration(ctx);
    return std::make_unique<StateAccelerating>();
  }
  else {
    applyDeceleration(ctx);
    return std::make_unique<StateDecelerating>();
  }
}
DronePhase StateDecelerating::name() const
{
  return DECELERATING;
}

std::unique_ptr<IDroneState> StateTurning::execute(MissionContext& ctx)
{
  if (std::abs(ctx.angleDiff) < ctx.turnThreshold) {
    ctx.remainingTurnTime = 0.0f;
    applyAcceleration(ctx);
    return std::make_unique<StateAccelerating>();
  }
  else if (ctx.remainingTurnTime <= 0.0f) {
    ctx.remainingTurnTime = std::abs(ctx.angleDiff) / ctx.angularSpeed;
    applyTurnInPlace(ctx);
    return std::make_unique<StateTurning>();
  }
  else {
    applyTurnInPlace(ctx);
    return std::make_unique<StateTurning>();
  }
}
DronePhase StateTurning::name() const
{
  return TURNING;
}
