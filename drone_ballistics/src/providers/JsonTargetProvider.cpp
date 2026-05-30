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

  this->targets.targetCount = targetsRaw["targetCount"];
  this->targets.timeSteps = targetsRaw["timeSteps"];

  this->targets.positions = new Coord *[this->targets.targetCount];
  for (int i = 0; i < this->targets.targetCount; i++) {
    // Allocate each row
    this->targets.positions[i] = new Coord[this->targets.timeSteps];

    // Get a reference to the specific target's positions array
    const auto &jsonPositions = targetsRaw["targets"][i]["positions"];

    for (int j = 0; j < this->targets.timeSteps; j++) {
      // Access the specific position object once
      const auto &pos = jsonPositions[j];

      this->targets.positions[i][j].x = pos["x"].get<float>();
      this->targets.positions[i][j].y = pos["y"].get<float>();
    }
  };
}

int JsonTargetProvider::getTargetCount()
{
  return this->targets.targetCount;
}

int JsonTargetProvider::getTimeSteps()
{
  return this->targets.timeSteps;
}

Coord *JsonTargetProvider::getTarget(int idx)
{
  return this->targets.positions[idx];
}

JsonTargetProvider::~JsonTargetProvider()
{
  // Clean up allocated memory
  for (int i = 0; i < this->targets.targetCount; i++) {
    delete[] this->targets.positions[i];
  }
  delete[] this->targets.positions;
}