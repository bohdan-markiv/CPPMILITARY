#pragma once
#include <vector>
#include "Types.h"
#include "interfaces/ITargetProvider.h"

class JsonTargetProvider : public ITargetProvider {
private:
  std::vector<std::vector<Coord>> targets;
  int TargetCount;
  int TimeSteps;

public:
  JsonTargetProvider() = default;
  JsonTargetProvider(const JsonTargetProvider &) = delete;
  JsonTargetProvider &operator=(const JsonTargetProvider &) = delete;
  void load() override;
  int getTargetCount() override;
  int getTimeSteps() override;
  Coord *getTarget(int idx) override;
};