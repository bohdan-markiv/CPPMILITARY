#pragma once

#include "Types.h"

float distanceCalculation(Coord currentCoord, Coord targetCoord);

float timeToTarget(float m, float d, float l, float attackSpeed, float zd);

float ammoFlyDistance(float t, float d, float l, float attackSpeed, float m);

void interpolateTarget(float t, float arrayTimeStep, Coord *trajectory, int timeSteps, Coord &interpolatedCoord);