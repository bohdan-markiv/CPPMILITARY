#include "Types.h"
#include "MissionProcessor.h"
#include "config/ComponentFactory.h"
#include <iostream>
#include <fstream>
#include <thread>
#include <chrono>
#include "threads/DronePhysics.h"

std::vector<SimStep> simLog;

auto main(int argc, char **argv) -> int
{
  SolverType solverType;
  if (argc > 1 && std::string(argv[1]) == "table") {
    solverType = SolverType::TABLE;
  }
  else if (argc > 1 && std::string(argv[1]) == "analytical") {
    solverType = SolverType::ANALYTICAL;
  }
  else {
    std::cerr << "Usage: " << argv[0] << " [table|analytical]\n";
    return 1;
  }
  auto configLoader = createLoader(LoaderType::FILE);
  auto targetProvider = createProvider(ProviderType::JSON);
  auto ballisticSolver = createSolver(solverType);

  Mission mission(std::move(ballisticSolver), std::move(targetProvider), std::move(configLoader));

  try {
    mission.init();
  }
  catch (const std::exception &e) {
    std::cerr << "Error during initialization: " << e.what() << std::endl;
    return 1;
  }

  try {
    // spawn the three worker threads
    std::thread providerThread(&ITargetProvider::run, mission.getProvider());
    std::thread physicsThread(&DronePhysics::run, mission.getPhysics());
    std::thread missionThread(&Mission::run, &mission);

    // wait until all three are warmed up
    while (!mission.getProvider()->isThreadReady() || !mission.getPhysics()->isThreadReady() || !mission.isThreadReady()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    // release them simultaneously
    mission.getProvider()->start();
    mission.getPhysics()->start();
    mission.start();

    // mission is the one we wait on
    missionThread.join();
    fprintf(stderr, "[main] missionThread joined\n");

    // mission finished → stop the others
    mission.getPhysics()->stop();   // sets flag, wakes queue, joins internally
    mission.getProvider()->stop();  // sets flag, joins internally
    physicsThread.join();
    fprintf(stderr, "[main] missionThread joined\n");
    providerThread.join();
    fprintf(stderr, "[main] missionThread joined\n");
  }
  catch (const std::exception &e) {
    std::cerr << "Error during simulation: " << e.what() << std::endl;
    return 1;
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
      stepJson["timeSecSinceStart"] = s.timeSecSinceStart;  // <-- ADD THIS
      out["steps"].push_back(stepJson);
    }

    std::ofstream outFile("drone_ballistics/output/simulation.json");
    if (!outFile.is_open()) {
      throw std::runtime_error("Cannot open output/simulation.json — does the folder exist?");
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
