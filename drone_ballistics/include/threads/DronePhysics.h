#pragma once
#include <mutex>
#include "Types.h"
#include "ThreadSafeQueue.h"

class DronePhysics {
public:
  void init(Coord startPos, float initialDir, float attackSpeed, float accel, float angularSpeed, float turnThreshold);
  void pushCommand(DroneCommand command);
  void step(float dt);
  DroneTelemetry getTelemetry() const;

private:
  Coord pos_{0.0f, 0.0f};
  float direction_ = 0.0f;  // in radians
  float currentSpeed_ = 0.0f;
  float remainingTurnTime_ = 0.0f;
  float timeSec_ = 0.0f;

  float attackSpeed_ = 0.0f;
  float a_ = 0.0f;
  float angularSpeed_ = 0.0f;
  float dt_ = 0.0f;

  void applyCruise();
  void applyTurnInPlace(float angleDiff);
  void applyDeceleration(float angleDiff);
  void applySmallTurn(float angleDiff);
  void applyMoving(float angleDiff);
  void applyAcceleration(float angleDiff);

  ThreadSafeQueue<DroneCommand> commandQueue_;
  mutable std::mutex mutex_;
};