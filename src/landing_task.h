#ifndef ODOM_TO_MAVROS_LANDING_TASK_H
#define ODOM_TO_MAVROS_LANDING_TASK_H

#include <string>

// One process lifetime represents one flight. Recovery/failure never cancels
// a pending request; only successful sending completes the task.
class LandingTask
{
public:
  bool latch(const std::string& reason)
  {
    if (state_ != State::Monitoring) return false;
    reason_ = reason;
    state_ = State::Pending;
    return true;
  }
  void markSent()
  {
    if (state_ == State::Pending) state_ = State::Sent;
  }
  bool pending() const { return state_ == State::Pending; }
  bool sent() const { return state_ == State::Sent; }
  bool triggered() const { return state_ != State::Monitoring; }
  const std::string& reason() const { return reason_; }

private:
  enum class State { Monitoring, Pending, Sent };
  State state_ = State::Monitoring;
  std::string reason_;
};
#endif
