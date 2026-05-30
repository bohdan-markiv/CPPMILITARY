#include "Types.h"
#include "interfaces/IBallisticsSolver.h"
#include "interfaces/ITargetProvider.h"
#include "interfaces/IConfigLoader.h"
class Mission {
  IBallisticSolver *solver;
  ITargetProvider *targets;
  IConfigLoader *configs;

  DroneConfig config;
  AmmoParams ammo;
  int currentIteration = 0;
  int currentIdx = 0;
  int targetCount = 0;

  SimStep simulation;

  double t = 0.0;

  // OUTPUT  arrays
  int N = 0;
  bool TARGET_HIT = false;

  double currentSpeed = 0.0;
  double remainingTurnTime = 0.0;

  double t_ammo = 0.0;
  double h_ammo = 0.0;
  double a = 0.0;
  int timeSteps = 0;

public:
  Mission(IBallisticSolver *solver, ITargetProvider *targets, IConfigLoader *configs)
    : solver(solver)
    , targets(targets)
    , configs(configs)
  {
  }

  float calculateTimeToStop(float currentSpeed, float attackSpeed, bool targetChanged, DronePhase phase, float remainingTurnTime, float a);
  float angleDifference(float from, float to);
  void init();
  bool hasNext();
  int getCurrentTargetIdx();
  void reset();
  void changeSolver(IBallisticSolver *newSolver);
  SimStep step();
  [[nodiscard]] int getN() const;
};