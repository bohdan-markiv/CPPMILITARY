#pragma once

#include "Types.h"
#include "state/MissionContext.h"

float distanceCalculation(Coord currentCoord, Coord targetCoord);

float timeToTarget(float m, float d, float l, float attackSpeed, float zd);

float ammoFlyDistance(float t, float d, float l, float attackSpeed, float m);

void interpolateTarget(float t, float arrayTimeStep, Coord* trajectory, int timeSteps, Coord& interpolatedCoord);

void applyTurnInPlace(MissionContext& ctx);
void applyCruise(MissionContext& ctx);
void applyDeceleration(MissionContext& ctx);
void applySmallTurn(MissionContext& ctx);
void applyAcceleration(MissionContext& ctx);
void applyMoving(MissionContext& ctx);