#include "Types.h"
#include "MissionProcessor.h"
#include "config/ComponentFactory.h"

auto main() -> int
{
  IConfigLoader *configLoader = createLoader(LoaderType::FILE);
  ITargetProvider *targetProvider = createProvider(ProviderType::JSON);
  IBallisticSolver *ballisticSolver = createSolver(SolverType::ANALYTICAL);

  Mission mission(ballisticSolver, targetProvider, configLoader);

  mission.init();

  while (mission.hasNext()) {
    SimStep step = mission.step();
    std::cout << "Step " << mission.getN() << " pos=(" << step.pos.x << "," << step.pos.y << ")" << " dir=" << step.direction
              << " state=" << step.state << " target=" << step.targetIdx << "\n";
    simLog.push_back(step);
  }

  json out;
  out["totalSteps"] = simLog.size();
  out["steps"] = json::array();
  for (const SimStep &s : simLog) {
    json stepJson;
    stepJson["position"] = {{"x", s.pos.x}, {"y", s.pos.y}};
    stepJson["direction"] = s.direction;
    stepJson["state"] = (int)s.state;
    stepJson["targetIndex"] = s.targetIdx;
    stepJson["dropPoint"] = {{"x", s.dropPoint.x}, {"y", s.dropPoint.y}};
    stepJson["aimPoint"] = {{"x", s.aimPoint.x}, {"y", s.aimPoint.y}};
    stepJson["predictedTarget"] = {{"x", s.predictedTarget.x}, {"y", s.predictedTarget.y}};
    out["steps"].push_back(stepJson);
  }

  std::ofstream outFile("drone_ballistics/output/simulation.json");
  if (!outFile.is_open()) {
    throw std::runtime_error("Cannot open output/simulation.json — does the folder exist?");
  }
  outFile << out.dump(2);
  outFile.close();

  delete configLoader;
  delete targetProvider;
  delete ballisticSolver;

  return 0;
}
