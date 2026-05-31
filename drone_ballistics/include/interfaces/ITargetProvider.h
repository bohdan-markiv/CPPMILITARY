#pragma once
#include "Types.h"

class ITargetProvider {
public:
  virtual ~ITargetProvider() {}
  virtual void load() = 0;
  virtual int getTargetCount() = 0;
  virtual int getTimeSteps() = 0;
  virtual Coord *getTarget(int idx) = 0;
};