#include "Types.h"
#include "MissionProcessor.h"
#include "config/ComponentFactory.h"
#include <iostream>
#include <fstream>
#include "telemetry/MavlinkLink.hpp"
#include <chrono>
#include <thread>
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
  std::string host = "127.0.0.1";
  std::uint16_t port = 14550;
  if (argc > 2 && !telemetry::parse_endpoint(argv[2], host, port)) {
    std::cerr << "bad endpoint: " << argv[2] << " (expected host:port)\n";
    return 1;
  }
  std::cout << "telemetry -> " << host << ":" << port << "\n";

  telemetry::UdpLink link(host, port);
  try {
    mission.init();
  }
  catch (const std::exception &e) {
    std::cerr << "Error during initialization: " << e.what() << std::endl;
    return 1;
  }
  if (!link.ok())
    return 1;
  telemetry::DropCommander drop(link);
  bool drop_requested = false;
  int tick_count = 0;
  const auto dt = std::chrono::milliseconds(100);  // matches simTimeStep = 0.1 s
  auto next_tick = std::chrono::steady_clock::now();
  while (mission.hasNext()) {
    try {
      SimStep step = mission.step();
      if (tick_count++ % 10 == 0) {
        telemetry::send_heartbeat(link);
      }
      telemetry::TelemetryState ts;
      ts.x_east_m = step.pos.x;
      ts.y_north_m = step.pos.y;
      ts.alt_m = mission.getAltitude();
      ts.speed_ms = mission.getSpeed();
      ts.direction_rad = step.direction;
      ts.time_boot_ms = static_cast<std::uint32_t>(mission.getT() * 1000.0F);
      telemetry::send_telemetry(link, ts);
      const auto now_ms = static_cast<std::uint32_t>(mission.getT() * 1000.0F);
      if (!drop_requested && mission.targetHit()) {
        const auto g = telemetry::local_to_geo(step.pos.x, step.pos.y);
        drop.request(now_ms, g.lat_deg, g.lon_deg, mission.getAltitude());
        drop_requested = true;
      }
      drop.tick(now_ms);
      next_tick += dt;
      std::this_thread::sleep_until(next_tick);
      std::cout << "Step " << mission.getN() << " pos=(" << step.pos.x << "," << step.pos.y << ")" << " dir=" << step.direction
                << " state=" << step.state << " target=" << step.targetIdx << "\n";
      simLog.push_back(step);
    }
    catch (const std::exception &e) {
      std::cerr << "Error during simulation step: " << e.what() << std::endl;
      return 1;
    }
  }
  // The drop fires on the final step, which also ends the flight. Keep the
  // link alive so retries can run and telemetry does not stop mid-sequence.
  if (drop.status() == telemetry::DropStatus::Pending) {
    const SimStep &last = simLog.back();
    float t_ms = mission.getT() * 1000.0F;

    while (drop.status() == telemetry::DropStatus::Pending) {
      t_ms += 100.0F;
      const auto now_ms = static_cast<std::uint32_t>(t_ms);

      telemetry::TelemetryState ts;
      ts.x_east_m = last.pos.x;
      ts.y_north_m = last.pos.y;
      ts.alt_m = mission.getAltitude();
      ts.speed_ms = 0.0;
      ts.direction_rad = last.direction;
      ts.time_boot_ms = now_ms;

      if (tick_count++ % 10 == 0) {
        telemetry::send_heartbeat(link);
      }
      telemetry::send_telemetry(link, ts);
      drop.tick(now_ms);

      next_tick += dt;
      std::this_thread::sleep_until(next_tick);
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
