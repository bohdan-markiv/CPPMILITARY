#pragma once

#include "interfaces/ITargetProvider.h"
#include "Types.h"

struct MissionContext {
  Coord pos;
  float direction;
  float currentSpeed;
  float remainingTurnTime;

  float angleDiff;

  float timeToStop;
  float a;
  float attackSpeed;

  float angularSpeed;
  float turnThreshold;

  float simTimeStep;
  Coord targetCoord;

  float h_ammo;
  float t_ammo;
  float arrayTimeStep;
  int targetIdx;

  Coord hitCoord;
  float t;
  int N;
  ITargetProvider* targets;
  int timeSteps;
  float hitRadius;

  bool targetHit = false;
  Coord predictedTarget;
};
