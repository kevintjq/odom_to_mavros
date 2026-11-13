#ifndef ODOM_TO_MAVROS_SAFETY_MONITOR_H
#define ODOM_TO_MAVROS_SAFETY_MONITOR_H

#include <algorithm>
#include <cmath>
#include <deque>
#include <mutex>

// Input-only health checks. All time arguments use a monotonic clock.
class SafetyMonitor
{
public:
  struct Config
  {
    double measurement_timeout = 0.2;
    double invalid_duration = 0.05;
    unsigned consecutive_invalid_count = 3;
    double startup_timeout = 5.0;
    double quality_window = 0.2;
    double max_invalid_ratio = 0.7;
    unsigned quality_min_samples = 10;
  };
  enum class Fault { None, StartupTimeout, Timeout, InvalidSequence, PoorQuality };

  SafetyMonitor(const Config& config, double now)
    : config_(config), startup_deadline_(now + config.startup_timeout) {}

  bool accept(double now, double measurement_age)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!std::isfinite(measurement_age) || measurement_age < 0.0 ||
        measurement_age >= config_.measurement_timeout)
      return false;
    // Delay already consumed part of this measurement's usable lifetime.
    const double deadline = now + config_.measurement_timeout - measurement_age;
    deadline_ = active_ ? std::max(deadline_, deadline) : deadline;
    active_ = true;
    consecutive_invalid_ = 0;
    recordSample(now, true);  // One good sample does not erase the quality window.
    return true;
  }

  void reject(double now)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (consecutive_invalid_ == 0 || now - last_invalid_ >= config_.measurement_timeout) {
      consecutive_invalid_ = 0;
      first_invalid_ = now;
    }
    if (consecutive_invalid_ < config_.consecutive_invalid_count)
      ++consecutive_invalid_;
    last_invalid_ = now;
    recordSample(now, false);
  }

  Fault fault(double now)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    pruneSamples(now);
    if (!active_)
      return now >= startup_deadline_ ? Fault::StartupTimeout : Fault::None;
    if (now >= deadline_) return Fault::Timeout;
    if (consecutive_invalid_ >= config_.consecutive_invalid_count &&
        now - first_invalid_ >= config_.invalid_duration)
      return Fault::InvalidSequence;
    if (samples_.size() >= config_.quality_min_samples &&
        samples_.back().time - samples_.front().time >= config_.invalid_duration &&
        static_cast<double>(invalid_samples_) / samples_.size() >= config_.max_invalid_ratio)
      return Fault::PoorQuality;
    return Fault::None;
  }

private:
  struct Sample { double time; bool valid; };
  void pruneSamples(double now)
  {
    while (!samples_.empty() && now - samples_.front().time > config_.quality_window) {
      if (!samples_.front().valid) --invalid_samples_;
      samples_.pop_front();
    }
  }
  void recordSample(double now, bool valid)
  {
    pruneSamples(now);
    // Bound memory even if a faulty source floods the callback queue.
    if (samples_.size() >= 4096) {
      if (!samples_.front().valid) --invalid_samples_;
      samples_.pop_front();
    }
    samples_.push_back({now, valid});
    if (!valid) ++invalid_samples_;
  }

  const Config config_;
  const double startup_deadline_;
  std::mutex mutex_;
  std::deque<Sample> samples_;
  unsigned invalid_samples_ = 0;
  bool active_ = false;
  double deadline_ = 0.0;
  double first_invalid_ = 0.0;
  double last_invalid_ = 0.0;
  unsigned consecutive_invalid_ = 0;
};
#endif
