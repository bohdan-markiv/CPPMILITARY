#pragma once
#include "Types.h"

class ITargetProvider {
public:
  virtual void load() = 0;
  virtual int getTargetCount() = 0;
  virtual int getTimeSteps() = 0;
  virtual Coord *getTarget(int idx) = 0;
  virtual ~ITargetProvider() = default;
};