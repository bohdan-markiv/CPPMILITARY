#pragma once
#include "Types.h"

class IBallisticSolver {
public:
  virtual ~IBallisticSolver() = default;
  virtual Coord solve(Coord currentCoord, float zd, Coord targetCoord, float attackSpeed, double m, double d, double l) = 0;
  virtual Result ammoFlight(float Z0, float V0, float m, float d, float l) const = 0;  // add
};