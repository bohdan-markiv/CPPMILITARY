#pragma once
#include "Types.h"
#include "state/MissionContext.h"

class IDroneState {
public:
  virtual ~IDroneState() = default;
  virtual std::unique_ptr<IDroneState> execute(MissionContext& ctx) = 0;
  virtual DronePhase name() const = 0;
};