#pragma once
#include "Types.h"
#include "interfaces/IBallisticsSolver.h"
#include "solvers/BallisticTable.h"
class TableSolver : public IBallisticSolver {
  BallisticTable table;

public:
  TableSolver() { table.load("drone_ballistics/data/ballistic_table.txt"); }
  Coord solve(Coord currentCoord, float zd, Coord targetCoord, float attackSpeed, double m, double d, double l) override;
};