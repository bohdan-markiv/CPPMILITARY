#pragma once
#include <vector>
#include "Types.h"
#include "interfaces/ITargetProvider.h"
#include <atomic>
#include <thread>
#include <mutex>

class JsonTargetProvider : public ITargetProvider {
private:
  std::vector<std::vector<Coord>> targets;
  int TargetCount;
  int TimeSteps;
  int currentNode = 0;
  float arrayTimeStep = 0.0f;

public:
  void run() override;
  bool isThreadReady() const override { return ready_.load(); }
  void start() override { started_.store(true); }
  void stop() override;
  void setTimeScale(float ts) override { timeScale_ = ts; }
  JsonTargetProvider() = default;
  JsonTargetProvider(const JsonTargetProvider &) = delete;
  JsonTargetProvider &operator=(const JsonTargetProvider &) = delete;
  void load() override;
  int getTargetCount() override;
  int getTimeSteps() override;
  Target getTarget(int idx) override;
  void advance() override;
  void setArrayTimeStep(float step) override;
  float timeScale_ = 1.0f;
  std::atomic<bool> ready_{false};
  std::atomic<bool> started_{false};
  std::atomic<bool> stop_{false};
  std::thread thread_;
  mutable std::mutex mutex_;
};