#pragma once
#include "Types.h"
#include "interfaces/ITargetProvider.h"

class JsonTargetProvider : public ITargetProvider {
private:
  TargetsConfig targets{0, 0, nullptr};

public:
  JsonTargetProvider() = default;
  JsonTargetProvider(const JsonTargetProvider &) = delete;
  JsonTargetProvider &operator=(const JsonTargetProvider &) = delete;
  void load() override;
  int getTargetCount() override;
  int getTimeSteps() override;
  Coord *getTarget(int idx) override;
  ~JsonTargetProvider() override;
};