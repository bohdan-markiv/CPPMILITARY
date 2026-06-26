#pragma once
#include <vector>
#include "Types.h"
#include "interfaces/ITargetProvider.h"

class JsonTargetProvider : public ITargetProvider {
private:
  std::vector<std::vector<Coord>> targets;
  int TargetCount;
  int TimeSteps;
  int currentNode = 0;
  float arrayTimeStep = 0.0f;

public:
  JsonTargetProvider() = default;
  JsonTargetProvider(const JsonTargetProvider &) = delete;
  JsonTargetProvider &operator=(const JsonTargetProvider &) = delete;
  void load() override;
  int getTargetCount() override;
  int getTimeSteps() override;
  Target getTarget(int idx) override;
  void advance() override;
  void setArrayTimeStep(float step) override;
};