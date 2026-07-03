#include <memory>
#include "Types.h"
#include "interfaces/IBallisticsSolver.h"
#include "interfaces/ITargetProvider.h"
#include "interfaces/IConfigLoader.h"
#include "interfaces/IDroneState.h"
#include "state/MissionContext.h"
class Mission {
  std::unique_ptr<IBallisticSolver> solver;
  std::unique_ptr<ITargetProvider> targets;
  std::unique_ptr<IConfigLoader> configs;
  std::unique_ptr<IDroneState> currentState;

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

  double kSwitchCost = 0.1;

  MissionContext ctx;

public:
  Mission(std::unique_ptr<IBallisticSolver> solver, std::unique_ptr<ITargetProvider> targets, std::unique_ptr<IConfigLoader> configs)
    : solver(std::move(solver))
    , targets(std::move(targets))
    , configs(std::move(configs))
  {
  }

  float calculateTimeToStop(float currentSpeed, float attackSpeed, bool targetChanged, DronePhase phase, float remainingTurnTime, float a);
  float angleDifference(float from, float to);
  void init();
  bool hasNext();
  int getCurrentTargetIdx();
  void reset();
  void changeSolver(std::unique_ptr<IBallisticSolver> newSolver);
  SimStep step();
  [[nodiscard]] int getN() const;
};