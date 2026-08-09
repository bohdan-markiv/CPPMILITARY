#include "Types.h"
#include "MissionProcessor.h"
#include "config/ComponentFactory.h"
#include <iostream>
#include <fstream>
#include <filesystem>
std::vector<SimStep> simLog;

auto main(int argc, char **argv) -> int
{
  SolverType solverType;
  if (argc < 3) {
    std::cerr << "Usage: " << argv[0] << " [table|analytical] <testId>\n";
    return 1;
  }
  const std::string testId = argv[2];

  SolverType solverType;
  if (std::string(argv[1]) == "table") {
    solverType = SolverType::TABLE;
  }
  else if (std::string(argv[1]) == "analytical") {
    solverType = SolverType::ANALYTICAL;
  }
  else {
    std::cerr << "Usage: " << argv[0] << " [table|analytical] <testId>\n";
    return 1;
  }
  const std::string testId = argv[2];
  auto configLoader = createLoader(LoaderType::FILE);
  auto targetProvider = createProvider(ProviderType::JSON, "drone_ballistics/data/" + testId + "/targets.json");
  auto ballisticSolver = createSolver(solverType);

  Mission mission(std::move(ballisticSolver), std::move(targetProvider), std::move(configLoader));

  try {
    mission.init();
  }
  catch (const std::exception &e) {
    std::cerr << "Error during initialization: " << e.what() << std::endl;
    return 1;
  }

  while (mission.hasNext()) {
    try {
      SimStep step = mission.step();
      std::cout << "Step " << mission.getN() << " pos=(" << step.pos.x << "," << step.pos.y << ")" << " dir=" << step.direction
                << " state=" << step.state << " target=" << step.targetIdx << "\n";
      simLog.push_back(step);
    }
    catch (const std::exception &e) {
      std::cerr << "Error during simulation step: " << e.what() << std::endl;
      return 1;
    }
  }

  try {
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

    const std::filesystem::path outDir = std::filesystem::path("drone_ballistics/output") / testId;
    std::filesystem::create_directories(outDir);
    std::ofstream outFile(outDir / "simulation.json");
    if (!outFile.is_open()) {
      throw std::runtime_error("Cannot open " + (outDir / "simulation.json").string());
    }
    outFile << out.dump(2);
    outFile.close();
  }
  catch (const std::exception &e) {
    std::cerr << "Error during output generation: " << e.what() << std::endl;
    return 1;
  }

  return 0;
}
