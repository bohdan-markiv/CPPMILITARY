#pragma once
#include "Types.h"

class ITargetProvider {
public:
  virtual void load() = 0;
  virtual int getTargetCount() = 0;
  virtual int getTimeSteps() = 0;
  virtual Target getTarget(int idx) = 0;
  virtual void setArrayTimeStep(float step) = 0;
  virtual ~ITargetProvider() = default;
  virtual void advance() = 0;
  virtual void run() = 0;
  virtual bool isThreadReady() const = 0;
  virtual void start() = 0;
  virtual void stop() = 0;
  virtual void setTimeScale(float ts) = 0;
};