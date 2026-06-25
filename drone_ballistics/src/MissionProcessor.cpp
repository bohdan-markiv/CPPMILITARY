#include "MissionProcessor.h"
#include <iostream>
#include "Types.h"
#include "helpers.h"
#include "state/StateClasses.h"

float Mission::calculateTimeToStop(
  float currentSpeed, float attackSpeed, bool targetChanged, DronePhase phase, float remainingTurnTime, float a)
{
  float fullAccelTime = attackSpeed / a;
  float timeToStop = 0.0f;

  if (targetChanged) {
    switch (phase) {
      case STOPPED:
        timeToStop = 0.0f;
        break;
      case ACCELERATING:
        timeToStop = currentSpeed / a;
        break;  // stop from current speed
      case MOVING:
        timeToStop = fullAccelTime;
        break;
      case DECELERATING:
        timeToStop = currentSpeed / a;
        break;
      case TURNING:
        timeToStop = remainingTurnTime;
        break;
    }
  }
  return timeToStop;
}

float Mission::angleDifference(float from, float to)
{
  float diff = fmod(to - from + kPi, 2.0f * kPi);
  if (diff < 0)
    diff += 2.0f * kPi;
  return diff - kPi;
}

void Mission::init()
{
  configs->load();
  targets->load();

  this->currentState = std::make_unique<StateStopped>();
  this->config = configs->getConfig();

  this->targets->setArrayTimeStep(config.arrayTimeStep);

  this->ammo = configs->getAmmoParams();
  this->targetCount = targets->getTargetCount();
  this->timeSteps = targets->getTimeSteps();
  fprintf(stderr, "timeSteps = %d, targetCount = %d\n", timeSteps, targetCount);
  simulation.pos = config.startPos;
  simulation.direction = config.initialDir;
  simulation.state = STOPPED;
  simulation.targetIdx = -1;
  simulation.dropPoint = {0.0f, 0.0f};
  simulation.aimPoint = {0.0f, 0.0f};
  simulation.predictedTarget = {0.0f, 0.0f};

  currentSpeed = 0.0f;
  remainingTurnTime = 0.0f;
  t = 0.0f;
  N = 0;
  TARGET_HIT = false;

  a = config.attackSpeed * config.attackSpeed / (2.0f * config.accelPath);
  Result flight = solver->ammoFlight(config.altitude, config.attackSpeed, ammo.mass, ammo.drag, ammo.lift);
  t_ammo = flight.t;
  h_ammo = flight.hDist;

  currentIteration = 0;
  currentIdx = 0;

  this->ctx.a = a;
  this->ctx.attackSpeed = config.attackSpeed;
  this->ctx.angularSpeed = config.angularSpeed;
  this->ctx.turnThreshold = config.turnThreshold;
  this->ctx.simTimeStep = config.simTimeStep;
  this->ctx.h_ammo = h_ammo;
  this->ctx.t_ammo = t_ammo;
  this->ctx.arrayTimeStep = config.arrayTimeStep;
  this->ctx.hitRadius = config.hitRadius;
  this->ctx.targets = targets.get();
  this->ctx.targetHit = false;
  this->ctx.pos = config.startPos;
  this->ctx.direction = config.initialDir;
  this->ctx.currentSpeed = 0.0f;
  this->ctx.remainingTurnTime = 0.0f;
  this->ctx.timeSteps = timeSteps;
}

bool Mission::hasNext()
{
  return !TARGET_HIT && N < MAX_STEPS;
}

int Mission::getCurrentTargetIdx()
{
  return this->currentIdx;
}

void Mission::reset()
{
  init();
}

void Mission::changeSolver(std::unique_ptr<IBallisticSolver> newSolver)
{
  this->solver = std::move(newSolver);
}

SimStep Mission::step()
{
  std::cout << "h" << h_ammo << " t " << t_ammo << std::endl;

  float bestTime = std::numeric_limits<float>::max();
  int chosenIdx = -1;
  Coord chosenDrop{0.0f, 0.0f};
  Coord chosenTargetPos{0.0f, 0.0f};
  Coord chosenPredicted{0.0f, 0.0f};

  for (int i = 0; i < targetCount; i++) {
    bool targetChanged = (i != simulation.targetIdx);

    Target tgt = targets->getTarget(i);
    Coord targetCoord = tgt.pos;
    float targetVx = tgt.velocity.x;
    float targetVy = tgt.velocity.y;

    Coord dropPoint{.x = 0.0f, .y = 0.0f};
    dropPoint = solver->solve(simulation.pos, config.altitude, targetCoord, config.attackSpeed, ammo.mass, ammo.drag, ammo.lift);

    double desiredDir = atan2(dropPoint.y - simulation.pos.y, dropPoint.x - simulation.pos.x);
    double angleDiff = angleDifference(simulation.direction, desiredDir);

    double startSpeed = currentSpeed;

    if (targetChanged) {
      switch (currentState->name()) {
        case STOPPED:
          startSpeed = 0.0f;
          break;
        case ACCELERATING:
          startSpeed = currentSpeed;
          break;  // continue accelerating from current speed
        case MOVING:
          startSpeed = config.attackSpeed;
          break;  // already at max speed
        case DECELERATING:
          startSpeed = currentSpeed;
          break;
        case TURNING:
          startSpeed = currentSpeed;
          break;  // maintain speed during turn
      }
    };

    double penaltyTime = 0.0;

    if (fabs(angleDiff) > config.turnThreshold) {
      double turningTime = fabs(angleDiff) / config.angularSpeed;
      penaltyTime += turningTime;
      startSpeed = 0.0;  // Assume we stop to turn
    }

    double timeToStop = calculateTimeToStop(currentSpeed, config.attackSpeed, targetChanged, currentState->name(), remainingTurnTime, a);
    penaltyTime += timeToStop;

    double timeToAccel = 0.0;
    double distanceDuringAccel = 0.0;
    if (startSpeed < config.attackSpeed) {
      timeToAccel = (config.attackSpeed - startSpeed) / a;
      distanceDuringAccel = startSpeed * timeToAccel + 0.5 * a * timeToAccel * timeToAccel;
    }

    penaltyTime += timeToAccel;

    double droneTravel = distanceCalculation(simulation.pos, dropPoint);
    double cruiseDistance = droneTravel - distanceDuringAccel;
    double totalTimeToTarget = penaltyTime + cruiseDistance / config.attackSpeed;

    Coord predictedCoord{.x = 0.0, .y = 0.0};
    for (int iter = 0; iter < 2; iter++) {
      predictedCoord.x = targetCoord.x + targetVx * (totalTimeToTarget + t_ammo);
      predictedCoord.y = targetCoord.y + targetVy * (totalTimeToTarget + t_ammo);
      dropPoint = solver->solve(simulation.pos, config.altitude, predictedCoord, config.attackSpeed, ammo.mass, ammo.drag, ammo.lift);
      droneTravel = distanceCalculation(simulation.pos, dropPoint);
      cruiseDistance = droneTravel - distanceDuringAccel;
      totalTimeToTarget = penaltyTime + cruiseDistance / config.attackSpeed;
    }

    if (totalTimeToTarget < 0.0f) {
      throw std::runtime_error("Calculated negative time to target, which is invalid.");
    }
    double effectiveTime = totalTimeToTarget;
    if (i == simulation.targetIdx) {
      effectiveTime *= (1.0 - kSwitchCost);
    }
    if (effectiveTime < bestTime) {
      bestTime = effectiveTime;
      chosenIdx = i;
      chosenDrop = dropPoint;
      chosenTargetPos = targetCoord;
      chosenPredicted = predictedCoord;
    }
  }

  simulation.targetIdx = chosenIdx;
  simulation.dropPoint = chosenDrop;
  simulation.predictedTarget = chosenPredicted;
  this->ctx.predictedTarget = chosenPredicted;
  interpolateTarget(t + t_ammo, config.arrayTimeStep, targets->getTarget(chosenIdx), timeSteps, simulation.aimPoint);

  Coord diffCoord{.x = simulation.dropPoint.x - simulation.pos.x, .y = simulation.dropPoint.y - simulation.pos.y};
  double desiredDir = atan2(diffCoord.y, diffCoord.x);
  double angleDiff = angleDifference(simulation.direction, desiredDir);
  // bool needBigTurn = fabs(angleDiff) > config.turnThreshold;

  ctx.pos = simulation.pos;
  ctx.direction = simulation.direction;
  ctx.currentSpeed = currentSpeed;
  ctx.remainingTurnTime = remainingTurnTime;
  ctx.angleDiff = angleDiff;
  ctx.t = t;
  ctx.N = N;
  ctx.targetIdx = simulation.targetIdx;

  auto next = currentState->execute(ctx);
  if (next)
    currentState = std::move(next);

  simulation.pos = ctx.pos;
  simulation.direction = ctx.direction;
  simulation.state = currentState->name();
  currentSpeed = ctx.currentSpeed;
  remainingTurnTime = ctx.remainingTurnTime;

  TARGET_HIT = ctx.targetHit;

  N++;
  t += config.simTimeStep;
  return simulation;
}
[[nodiscard]] int Mission::getN() const
{
  return N;
}