#include "MissionProcessor.h"
#include <iostream>
#include <memory>
#include "Types.h"
#include "helpers.h"
#include "state/StateClasses.h"
#include "threads/DronePhysics.h"
#include <thread>
#include <chrono>

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
  this->targets->setTimeScale(config.timeScale);

  this->ammo = configs->getAmmoParams();
  this->targetCount = targets->getTargetCount();
  this->targets->setArrayTimeStep(config.arrayTimeStep);
  this->timeSteps = targets->getTimeSteps();
  fprintf(stderr, "timeSteps = %d, targetCount = %d\n", timeSteps, targetCount);
  simulation.pos = config.startPos;
  simulation.direction = config.initialDir;
  simulation.state = STOPPED;
  simulation.targetIdx = -1;
  simulation.dropPoint = {0.0f, 0.0f};
  simulation.aimPoint = {0.0f, 0.0f};
  simulation.predictedTarget = {0.0f, 0.0f};
  simulation.timeSecSinceStart = 0.0f;

  currentSpeed = 0.0f;
  remainingTurnTime = 0.0f;
  t = 0.0f;
  N = 0;
  TARGET_HIT = false;

  a = config.attackSpeed * config.attackSpeed / (2.0f * config.accelPath);
  Result flight = solver->ammoFlight(config.altitude, config.attackSpeed, ammo.mass, ammo.drag, ammo.lift);
  t_ammo = flight.t;
  h_ammo = flight.hDist;

  physics = std::make_unique<DronePhysics>();
  physics->init(config.startPos, config.initialDir, config.attackSpeed, a, config.angularSpeed, config.physicsTimeStep);
  physics->setTimeScale(config.timeScale);
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
  // this->ctx.targets = targets.get();
  this->ctx.targetHit = false;
  this->ctx.pos = config.startPos;
  this->ctx.direction = config.initialDir;
  this->ctx.currentSpeed = 0.0f;
  this->ctx.remainingTurnTime = 0.0f;
  this->ctx.timeSteps = timeSteps;
  fprintf(stderr,
          "physicsTimeStep=%f simTimeStep=%f physicsSteps=%d\n",
          config.physicsTimeStep,
          config.simTimeStep,
          (int)std::round(config.simTimeStep / config.physicsTimeStep));
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
  DroneTelemetry tel = physics->getTelemetry();  // read drone state once, up front

  float bestTime = std::numeric_limits<float>::max();
  int chosenIdx = -1;
  Coord chosenDrop{0.0f, 0.0f};
  Coord chosenTargetPos{0.0f, 0.0f};
  Coord chosenPredicted{0.0f, 0.0f};

  for (int i = 0; i < targetCount; i++) {
    bool targetChanged = (i != simulation.targetIdx);

    Target tgt = targets->getTarget(i);  // by value, snapshot
    Coord targetCoord = tgt.pos;
    double targetVx = tgt.velocity.x;
    double targetVy = tgt.velocity.y;

    Coord dropPoint{0.0f, 0.0f};
    dropPoint = solver->solve(tel.pos, config.altitude, targetCoord, config.attackSpeed, ammo.mass, ammo.drag, ammo.lift);

    double desiredDir = atan2(dropPoint.y - tel.pos.y, dropPoint.x - tel.pos.x);
    double angleDiff = angleDifference(tel.direction, desiredDir);

    double startSpeed = tel.currentSpeed;

    if (targetChanged) {
      switch (currentState->name()) {
        case STOPPED:
          startSpeed = 0.0f;
          break;
        case ACCELERATING:
          startSpeed = tel.currentSpeed;
          break;
        case MOVING:
          startSpeed = config.attackSpeed;
          break;
        case DECELERATING:
          startSpeed = tel.currentSpeed;
          break;
        case TURNING:
          startSpeed = tel.currentSpeed;
          break;
      }
    }

    double penaltyTime = 0.0;

    if (fabs(angleDiff) > config.turnThreshold) {
      double turningTime = fabs(angleDiff) / config.angularSpeed;
      penaltyTime += turningTime;
      startSpeed = 0.0;
    }

    double timeToStop =
      calculateTimeToStop(tel.currentSpeed, config.attackSpeed, targetChanged, currentState->name(), tel.remainingTurnTime, a);
    penaltyTime += timeToStop;

    double timeToAccel = 0.0;
    double distanceDuringAccel = 0.0;
    if (startSpeed < config.attackSpeed) {
      timeToAccel = (config.attackSpeed - startSpeed) / a;
      distanceDuringAccel = startSpeed * timeToAccel + 0.5 * a * timeToAccel * timeToAccel;
    }

    penaltyTime += timeToAccel;

    double droneTravel = distanceCalculation(tel.pos, dropPoint);
    double cruiseDistance = droneTravel - distanceDuringAccel;
    double totalTimeToTarget = penaltyTime + cruiseDistance / config.attackSpeed;

    Coord predictedCoord{0.0, 0.0};
    for (int iter = 0; iter < 2; iter++) {
      predictedCoord.x = targetCoord.x + targetVx * (totalTimeToTarget + t_ammo);
      predictedCoord.y = targetCoord.y + targetVy * (totalTimeToTarget + t_ammo);
      dropPoint = solver->solve(tel.pos, config.altitude, predictedCoord, config.attackSpeed, ammo.mass, ammo.drag, ammo.lift);
      droneTravel = distanceCalculation(tel.pos, dropPoint);
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

  // aim-point: linear projection (trajectory matrix is hidden now)
  Target chosenTgt = targets->getTarget(chosenIdx);
  simulation.aimPoint = chosenTgt.pos + chosenTgt.velocity * t_ammo;

  Coord diffCoord{chosenDrop.x - tel.pos.x, chosenDrop.y - tel.pos.y};
  double desiredDir = atan2(diffCoord.y, diffCoord.x);
  double angleDiff = angleDifference(tel.direction, desiredDir);

  // fill ctx decision inputs from telemetry
  ctx.pos = tel.pos;
  ctx.direction = tel.direction;
  ctx.currentSpeed = tel.currentSpeed;
  ctx.remainingTurnTime = tel.remainingTurnTime;
  ctx.angleDiff = angleDiff;
  ctx.desiredDir = desiredDir;
  ctx.t = t;
  ctx.N = N;
  ctx.targetIdx = simulation.targetIdx;
  ctx.predictedTarget = chosenPredicted;

  auto next = currentState->execute(ctx);  // decision only — fills ctx.command
  if (next)
    currentState = std::move(next);

  physics->pushCommand(ctx.command);  // hand motion to physics
  // int physicsSteps = std::max(1, (int)std::round(config.simTimeStep / config.physicsTimeStep));  // 10
  // for (int k = 0; k < physicsSteps; k++) {
  //   physics->step(config.physicsTimeStep);
  // }

  // --- relocated hit-check: read post-integration telemetry ---
  DroneTelemetry post = physics->getTelemetry();

  simulation.timeSecSinceStart = post.timeSecSinceStart;

  if (currentState->name() == MOVING) {
    Coord bombLand;
    bombLand.x = post.pos.x + h_ammo * cos(post.direction);
    bombLand.y = post.pos.y + h_ammo * sin(post.direction);
    double missDistance = distanceCalculation(bombLand, chosenPredicted);
    if (missDistance <= config.hitRadius) {
      TARGET_HIT = true;
      ctx.targetHit = true;
      std::cout << "Target " << simulation.targetIdx << " HIT at t=" << t << "s (step " << N << ")" << std::endl;
      std::cout << "  Drone:  (" << post.pos.x << ", " << post.pos.y << ")  dir=" << post.direction << std::endl;
      std::cout << "  Bomb lands at: (" << bombLand.x << ", " << bombLand.y << ")" << std::endl;
      std::cout << "  Miss distance: " << missDistance << " m" << std::endl;
    }
  }

  // output from telemetry
  simulation.pos = post.pos;
  simulation.direction = post.direction;
  simulation.state = currentState->name();

  // provider advance — Phase-1 stand-in for the provider's own clock
  // if (config.simTimeStep > 0.0f) {
  //   int ticksPerNode = std::max(1, (int)std::round(config.arrayTimeStep / config.simTimeStep));
  //   if (N % ticksPerNode == 0)
  //     targets->advance();
  // }

  N++;
  t += config.simTimeStep;
  return simulation;
}
[[nodiscard]] int Mission::getN() const
{
  return N;
}
void Mission::run()
{
  ready_.store(true);
  while (!started_.load() && !stop_.load())
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  fprintf(stderr, "[mission] started loop\n");
  while (!stop_.load() && hasNext()) {
    SimStep s = step();
    simLog.push_back(s);
    if (N % 100 == 0)
      fprintf(stderr,
              "[mission] N=%d pos=(%.1f,%.1f) state=%d tIdx=%d\n",
              N,
              simulation.pos.x,
              simulation.pos.y,
              (int)simulation.state,
              simulation.targetIdx);
    std::this_thread::sleep_for(std::chrono::duration<float>(config.simTimeStep / config.timeScale));
  }
  fprintf(stderr, "[mission] loop exited at N=%d (hit=%d)\n", N, (int)TARGET_HIT);
  stop_.store(true);
}