#pragma once
#include "state/MissionContext.h"
#include "interfaces/IDroneState.h"

class StateStopped : public IDroneState {
public:
  std::unique_ptr<IDroneState> execute(MissionContext& ctx) override;
  DronePhase name() const override;
};

class StateAccelerating : public IDroneState {
public:
  std::unique_ptr<IDroneState> execute(MissionContext& ctx) override;
  DronePhase name() const override;
};

class StateMoving : public IDroneState {
public:
  std::unique_ptr<IDroneState> execute(MissionContext& ctx) override;
  DronePhase name() const override;
};

class StateDecelerating : public IDroneState {
public:
  std::unique_ptr<IDroneState> execute(MissionContext& ctx) override;
  DronePhase name() const override;
};

class StateTurning : public IDroneState {
public:
  std::unique_ptr<IDroneState> execute(MissionContext& ctx) override;
  DronePhase name() const override;
};