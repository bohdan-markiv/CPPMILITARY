#pragma once
#include <mutex>
#include "Types.h"
#include "ThreadSafeQueue.h"
#include <atomic>
#include <thread>

class DronePhysics {
public:
  void init(Coord startPos, float initialDir, float attackSpeed, float accel, float angularSpeed, float turnThreshold);
  void pushCommand(DroneCommand command);
  void step(float dt);
  DroneTelemetry getTelemetry() const;
  void run();
  bool isThreadReady() const { return ready_.load(); }
  void start() { started_.store(true); }
  void stop();
  void setTimeScale(float ts) { timeScale_ = ts; }

private:
  float timeScale_ = 1.0f;
  std::atomic<bool> ready_{false};
  std::atomic<bool> started_{false};
  std::atomic<bool> stop_{false};
  std::thread thread_;
  Coord pos_{0.0f, 0.0f};
  float direction_ = 0.0f;  // in radians
  float currentSpeed_ = 0.0f;
  float remainingTurnTime_ = 0.0f;
  float timeSec_ = 0.0f;

  float attackSpeed_ = 0.0f;
  float a_ = 0.0f;
  float angularSpeed_ = 0.0f;
  float dt_ = 0.0f;

  float targetDir_ = 0.0f;  // NEW

  static float angleDifference(float from, float to);  // NEW helper
  void applyCruise();
  void applyTurnInPlace();
  void applyDeceleration(float angleDiff);
  void applySmallTurn();
  void applyMoving();
  void applyAcceleration();

  ThreadSafeQueue<DroneCommand> commandQueue_;
  DroneCommand current_{MotionKind::None, 0.0f, -1.0f};
  mutable std::mutex mutex_;
};