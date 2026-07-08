#include "Types.h"
#include "providers/JsonTargetProvider.h"
#include <fstream>
#include <stdexcept>

void JsonTargetProvider::load()
{
  std::ifstream targetsFile("drone_ballistics/data/targets.json");
  if (!targetsFile.is_open()) {
    throw std::runtime_error("Cannot open targets.json");
  }
  json targetsRaw = json::parse(targetsFile);

  this->TargetCount = targetsRaw["targetCount"];
  this->TimeSteps = targetsRaw["timeSteps"];

  this->targets.resize(this->TargetCount);
  for (int i = 0; i < this->TargetCount; i++) {
    this->targets[i].resize(this->TimeSteps);

    // Get a reference to the specific target's positions array
    const auto &jsonPositions = targetsRaw["targets"][i]["positions"];

    for (int j = 0; j < this->TimeSteps; j++) {
      // Access the specific position object once
      const auto &pos = jsonPositions[j];

      this->targets[i][j].x = pos["x"].get<float>();
      this->targets[i][j].y = pos["y"].get<float>();
    }
  };
}

int JsonTargetProvider::getTargetCount()
{
  return this->TargetCount;
}

int JsonTargetProvider::getTimeSteps()
{
  return this->TimeSteps;
}

Target JsonTargetProvider::getTarget(int idx)
{
  std::lock_guard<std::mutex> lock(mutex_);
  Target out;
  out.pos = targets[idx][currentNode];
  int nextNode = (currentNode + 1) % TimeSteps;
  Coord d = targets[idx][nextNode] - targets[idx][currentNode];
  out.velocity = d / arrayTimeStep;
  return out;
}

void JsonTargetProvider::advance()
{
  currentNode = (currentNode + 1) % this->TimeSteps;
}

void JsonTargetProvider::setArrayTimeStep(float step)
{
  this->arrayTimeStep = step;
}

void JsonTargetProvider::run()
{
  ready_.store(true);
  while (!started_.load() && !stop_.load())
    std::this_thread::sleep_for(std::chrono::milliseconds(1));

  while (!stop_.load()) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      currentNode = (currentNode + 1) % TimeSteps;
    }
    std::this_thread::sleep_for(std::chrono::duration<float>(arrayTimeStep / timeScale_));  // arrayTimeStep, not targetTimeStep_
  }
}
void JsonTargetProvider::stop()
{
  stop_.store(true);
  if (thread_.joinable())
    thread_.join();
  fprintf(stderr, "[provider] stop called\n");
}