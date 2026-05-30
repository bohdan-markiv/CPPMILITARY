#include "MissionProcessor.h"
#include "Types.h"
#include "helpers.h"

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

  this->config = configs->getConfig();
  this->ammo = configs->getAmmoParams();
  this->targetCount = targets->getTargetCount();
  this->timeSteps = targets->getTimeSteps();

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
  t_ammo = timeToTarget(ammo.mass, ammo.drag, ammo.lift, config.attackSpeed, config.altitude);
  h_ammo = ammoFlyDistance(t_ammo, ammo.drag, ammo.lift, config.attackSpeed, ammo.mass);

  currentIteration = 0;
  currentIdx = 0;
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

void Mission::changeSolver(IBallisticSolver *newSolver)
{
  this->solver = newSolver;
}

SimStep Mission::step()
{
  float bestTime = std::numeric_limits<float>::max();
  int chosenIdx = -1;
  Coord chosenDrop{0.0f, 0.0f};
  Coord chosenTargetPos{0.0f, 0.0f};

  for (int i = 0; i < targetCount; i++) {
    bool targetChanged = (i != simulation.targetIdx);

    Coord targetCoord{0.0f, 0.0f}, nextCoord{0.0f, 0.0f};
    interpolateTarget(t, config.arrayTimeStep, targets->getTarget(i), timeSteps, targetCoord);
    interpolateTarget(t + config.simTimeStep, config.arrayTimeStep, targets->getTarget(i), timeSteps, nextCoord);
    Coord dCoord = nextCoord - targetCoord;
    double targetVx = dCoord.x / config.simTimeStep;
    double targetVy = dCoord.y / config.simTimeStep;

    Coord dropPoint{.x = 0.0f, .y = 0.0f};
    dropPoint = solver->solve(simulation.pos, config.altitude, targetCoord, config.attackSpeed, ammo.mass, ammo.drag, ammo.lift);

    double desiredDir = atan2(dropPoint.y - simulation.pos.y, dropPoint.x - simulation.pos.x);
    double angleDiff = angleDifference(simulation.direction, desiredDir);

    double startSpeed = currentSpeed;

    if (targetChanged) {
      switch (simulation.state) {
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

    double timeToStop = calculateTimeToStop(currentSpeed, config.attackSpeed, targetChanged, simulation.state, remainingTurnTime, a);
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
    if (totalTimeToTarget < bestTime) {
      bestTime = totalTimeToTarget;
      chosenIdx = i;
      chosenDrop = dropPoint;
      chosenTargetPos = targetCoord;
    }
  }

  simulation.targetIdx = chosenIdx;
  simulation.dropPoint = chosenDrop;
  simulation.predictedTarget = chosenTargetPos;

  interpolateTarget(t + t_ammo, config.arrayTimeStep, targets->getTarget(chosenIdx), timeSteps, simulation.aimPoint);

  Coord diffCoord{.x = simulation.dropPoint.x - simulation.pos.x, .y = simulation.dropPoint.y - simulation.pos.y};
  double desiredDir = atan2(diffCoord.y, diffCoord.x);
  double angleDiff = angleDifference(simulation.direction, desiredDir);
  bool needBigTurn = fabs(angleDiff) > config.turnThreshold;

  switch (simulation.state) {
    case STOPPED:
      if (needBigTurn) {
        simulation.state = TURNING;
        remainingTurnTime = fabs(angleDiff) / config.angularSpeed;
      }
      else {
        simulation.state = ACCELERATING;
      }
      break;

    case ACCELERATING:
      if (needBigTurn) {
        simulation.state = DECELERATING;
      }
      else if (currentSpeed >= config.attackSpeed) {
        simulation.state = MOVING;
        currentSpeed = config.attackSpeed;  // Ensure we don't exceed max speed
      }
      break;

    case MOVING:
      if (needBigTurn) {
        simulation.state = DECELERATING;
      }
      break;

    case DECELERATING:
      if (currentSpeed <= 0.0f) {
        currentSpeed = 0.0f;
        simulation.state = STOPPED;  // next step will decide TURNING or ACCELERATING
      }
      else if (!needBigTurn) {
        // Turn requirement gone, re-accelerate
        simulation.state = ACCELERATING;
      }
      break;

    case TURNING:
      // Check if the angular difference is smaller than your threshold
      if (std::abs(angleDiff) < config.turnThreshold) {
        simulation.state = ACCELERATING;  // Safely move to acceleration
        remainingTurnTime = 0.0f;
      }
      else if (remainingTurnTime <= 0.0f) {
        remainingTurnTime = std::abs(angleDiff) / config.angularSpeed;
      }
      break;
  }

  switch (simulation.state) {
    case STOPPED:
      // no motion
      break;

    case ACCELERATING:
      currentSpeed += a * config.simTimeStep;
      if (currentSpeed > config.attackSpeed)
        currentSpeed = config.attackSpeed;
      // For small turns, adjust heading smoothly while moving
      if (!needBigTurn && fabs(angleDiff) > 1e-4f) {
        double turnStep = config.angularSpeed * config.simTimeStep;
        if (turnStep > fabs(angleDiff))
          turnStep = fabs(angleDiff);
        simulation.direction += (angleDiff > 0 ? 1.0 : -1.0) * turnStep;
      }
      // Update position
      simulation.pos.x += currentSpeed * cos(simulation.direction) * config.simTimeStep;
      simulation.pos.y += currentSpeed * sin(simulation.direction) * config.simTimeStep;
      break;

    case MOVING:
      if (!needBigTurn && fabs(angleDiff) > 1e-4f) {
        double turnStep = config.angularSpeed * config.simTimeStep;
        if (turnStep > fabs(angleDiff))
          turnStep = fabs(angleDiff);
        simulation.direction += (angleDiff > 0 ? 1.0 : -1.0) * turnStep;
      }
      simulation.pos.x += currentSpeed * cos(simulation.direction) * config.simTimeStep;
      simulation.pos.y += currentSpeed * sin(simulation.direction) * config.simTimeStep;
      break;

    case DECELERATING:
      currentSpeed -= a * config.simTimeStep;
      if (currentSpeed < 0.0)
        currentSpeed = 0.0;
      simulation.pos.x += currentSpeed * cos(simulation.direction) * config.simTimeStep;
      simulation.pos.y += currentSpeed * sin(simulation.direction) * config.simTimeStep;
      break;

    case TURNING: {
      // Turn in place — no position change
      double turnStep = config.angularSpeed * config.simTimeStep;
      if (turnStep > fabs(angleDiff))
        turnStep = fabs(angleDiff);
      simulation.direction += (angleDiff > 0 ? 1.0 : -1.0) * turnStep;
      remainingTurnTime -= config.simTimeStep;
      if (remainingTurnTime < 0.0)
        remainingTurnTime = 0.0;
    } break;
  }

  if (simulation.state == MOVING) {
    Coord bombLand{.x = 0.0, .y = 0.0};
    bombLand.x = simulation.pos.x + h_ammo * cos(simulation.direction);
    bombLand.y = simulation.pos.y + h_ammo * sin(simulation.direction);

    Coord hitCoord{.x = 0.0, .y = 0.0};
    interpolateTarget(t + t_ammo, config.arrayTimeStep, targets->getTarget(simulation.targetIdx), timeSteps, hitCoord);

    double missDistance = distanceCalculation(bombLand, hitCoord);

    if (missDistance <= config.hitRadius) {
      TARGET_HIT = true;
      std::cout << "Target " << simulation.targetIdx << " HIT at t=" << t << "s (step " << N << ")" << std::endl;
      std::cout << "  Drone:  (" << simulation.pos.x << ", " << simulation.pos.y << ")  dir=" << simulation.direction << std::endl;
      std::cout << "  Bomb lands at:   (" << bombLand.x << ", " << bombLand.y << ")" << std::endl;
      std::cout << "  Target at impact:(" << hitCoord.x << ", " << hitCoord.y << ")" << std::endl;
      std::cout << "  Miss distance:   " << missDistance << " m" << std::endl;
    }
  }

  N++;
  t += config.simTimeStep;
  return simulation;
}
[[nodiscard]] int Mission::getN() const
{
  return N;
}