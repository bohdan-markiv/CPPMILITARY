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
  void push(T value)
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      queue_.push(std::move(value));
    }
    condition_.notify_one();
  };

  bool try_pop(T& value)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (queue_.empty())
      return false;
    value = std::move(queue_.front());
    queue_.pop();
    return true;
  };

  void requestStop()
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stop_ = true;
    }
    condition_.notify_all();
  };

  bool wait_and_pop(T& value)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    condition_.wait(lock, [this] { return !queue_.empty() || stop_; });
    if (stop_ && queue_.empty())
      return false;
    value = std::move(queue_.front());
    queue_.pop();
    return true;
  };
};