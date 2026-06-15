#pragma once
#include "Types.h"
#include "interfaces/IBallisticsSolver.h"

class AnalyticalSolver : public IBallisticSolver {
public:
  Coord solve(Coord currentCoord, float zd, Coord targetCoord, float attackSpeed, double m, double d, double l) override;
  Result ammoFlight(float Z0, float V0, float m, float d, float l) const override;
};