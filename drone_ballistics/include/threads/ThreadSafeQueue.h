#pragma once
#include <queue>
#include <mutex>
#include <condition_variable>

template <typename T>
class ThreadSafeQueue {
private:
  std::queue<T> queue_;
  std::mutex mutex_;
  std::condition_variable condition_;
  bool stop_ = false;

public:
  void push(T value);

  bool try_pop(T& value);

  void requestStop();

  bool wait_and_pop(T& value);
};