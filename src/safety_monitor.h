#ifndef ODOM_TO_MAVROS_SAFETY_MONITOR_H
#define ODOM_TO_MAVROS_SAFETY_MONITOR_H

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <mutex>

// All time arguments are seconds from a monotonic clock. ROS time is used
// only to compute measurement_age at the bridge boundary.
class SafetyMonitor
{
public:
  enum class Fault { None, Timeout, InvalidSequence };
  SafetyMonitor(double timeout, double invalid_timeout, unsigned invalid_count)
    : timeout_(timeout), invalid_timeout_(invalid_timeout), min_invalid_(invalid_count) {}

  bool accept(double now, double measurement_age)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!std::isfinite(measurement_age) ||
        measurement_age < 0.0 || measurement_age >= timeout_)
      return false;
    // A delayed packet only provides the remaining part of its lifetime.
    const double deadline = now + timeout_ - measurement_age;
    deadline_ = active_ ? std::max(deadline_, deadline) : deadline;
    active_ = true;
    invalid_count_ = 0;
    return true;
  }

  void reject(double now)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    // Widely separated invalid packets are not a continuous burst.
    if (invalid_count_ == 0 || now - last_invalid_ >= timeout_) {
      invalid_count_ = 0;
      first_invalid_ = now;
    }
    if (invalid_count_ < min_invalid_) ++invalid_count_;
    last_invalid_ = now;
  }

  Fault fault(double now) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!active_) return Fault::None;  // Preserve the existing startup guard.
    if (invalid_count_ >= min_invalid_ && now - first_invalid_ >= invalid_timeout_)
      return Fault::InvalidSequence;
    if (now >= deadline_) return Fault::Timeout;
    return Fault::None;
  }

private:
  const double timeout_;
  const double invalid_timeout_;
  const unsigned min_invalid_;
  mutable std::mutex mutex_;
  bool active_ = false;
  double deadline_ = 0.0;
  double first_invalid_ = 0.0;
  double last_invalid_ = 0.0;
  unsigned invalid_count_ = 0;
};

#endif
