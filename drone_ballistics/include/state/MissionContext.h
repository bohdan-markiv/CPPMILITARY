#pragma once

#include "Types.h"

struct MissionContext {
  Coord pos;
  float direction;
  float currentSpeed;
  float remainingTurnTime;

  float angleDiff;
  float turnThreshold;
  float angularSpeed;
  float attackSpeed;
  float a;
  float simTimeStep;
  float arrayTimeStep;
  float h_ammo;
  float t_ammo;
  float timeToStop;
  float hitRadius;
  int timeSteps;
  float t;
  int N;
  int targetIdx;
  Coord predictedTarget;

  DroneCommand command;

  bool targetHit = false;
};
