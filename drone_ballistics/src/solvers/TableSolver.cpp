#include "Types.h"
#include "solvers/TableSolver.h"
#include <cstdio>
#include "solvers/BallisticTable.h"
#include "helpers.h"

Coord TableSolver::solve(Coord currentCoord, float zd, Coord targetCoord, float attackSpeed, double m, double d, double l)
{
  Coord fireCoord;
  double fullDistance = distanceCalculation(currentCoord, targetCoord);

  Result final = table.lookup(zd, attackSpeed, m, d, l);

  double ratio = (fullDistance > 0.0) ? std::max(0.0, (fullDistance - final.hDist) / fullDistance) : 0.0;
  fireCoord.x = currentCoord.x + (targetCoord.x - currentCoord.x) * ratio;
  fireCoord.y = currentCoord.y + (targetCoord.y - currentCoord.y) * ratio;

  return fireCoord;
}
Result TableSolver::ammoFlight(float Z0, float V0, float m, float d, float l) const
{
  return table.lookup(Z0, V0, m, d, l);
}