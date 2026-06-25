#include "threads/ThreadSafeQueue.h"

template <typename T>
void ThreadSafeQueue<T>::push(T value)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    queue_.push(std::move(value));
  }
  condition_.notify_one();
}
template <typename T>
bool ThreadSafeQueue<T>::try_pop(T& value)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (queue_.empty())
    return false;
  value = std::move(queue_.front());
  queue_.pop();
  return true;
}
template <typename T>
void ThreadSafeQueue<T>::requestStop()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stop_ = true;
  }
  condition_.notify_all();
}

template <typename T>
bool ThreadSafeQueue<T>::wait_and_pop(T& value)
{
  std::unique_lock<std::mutex> lock(mutex_);
  condition_.wait(lock, [this] { return !queue_.empty() || stop_; });
  if (stop_ && queue_.empty())
    return false;
  value = std::move(queue_.front());
  queue_.pop();
  return true;
}