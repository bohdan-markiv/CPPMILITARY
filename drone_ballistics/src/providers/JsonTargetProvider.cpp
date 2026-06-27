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

Coord *JsonTargetProvider::getTarget(int idx)
{
  return &this->targets[idx][0];
}
