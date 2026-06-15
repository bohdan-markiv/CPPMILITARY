#include "Types.h"
#include "solvers/AnalyticalSolver.h"
#include "helpers.h"

Coord AnalyticalSolver::solve(Coord currentCoord, float zd, Coord targetCoord, float attackSpeed, double m, double d, double l)
{
  Coord fireCoord;
  double fullDistance = distanceCalculation(currentCoord, targetCoord);

  double t = timeToTarget(m, d, l, attackSpeed, zd);
  if (t < 0.0) {
    throw std::runtime_error("Error calculating time to target.");
  }
  double h = ammoFlyDistance(t, d, l, attackSpeed, m);
  if (h < 0.0) {
    throw std::runtime_error("Error calculating ammo fly distance.");
  }

  double ratio = (fullDistance > 0.0) ? std::max(0.0, (fullDistance - h) / fullDistance) : 0.0;
  fireCoord.x = currentCoord.x + (targetCoord.x - currentCoord.x) * ratio;
  fireCoord.y = currentCoord.y + (targetCoord.y - currentCoord.y) * ratio;
  return fireCoord;
}
Result AnalyticalSolver::ammoFlight(float Z0, float V0, float m, float d, float l) const
{
  float t = timeToTarget(m, d, l, V0, Z0);
  float h = ammoFlyDistance(t, d, l, V0, m);
  return {t, h};
}