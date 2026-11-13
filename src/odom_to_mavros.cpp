#include <Eigen/Geometry>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/SetMode.h>
#include <nav_msgs/Odometry.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/spinner.h>
#include "safety_monitor.h"
#include <memory>
#include <mutex>

#include <future>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <algorithm>
#include <string>

class LioToMavros
{
public:
  LioToMavros(ros::NodeHandle& nh, ros::NodeHandle& pnh)
  {
    pnh.param<std::string>("input_topic", input_topic_, "/Odometry");
    pnh.param<std::string>("output_topic", output_topic_, "/mavros/vision_pose/pose");
    pnh.param<std::string>("frame_id", frame_id_, "world");
    pnh.param<double>("max_flight_speed", max_flight_speed_, 5.0);
    pnh.param<double>("max_angular_speed", max_angular_speed_deg_, 180.0);
    pnh.param<double>("vision_pose_timeout", vision_pose_timeout_sec_, 0.2);
    pnh.param<bool>("enable_vision_pose_timeout_protection",
                    enable_vision_pose_timeout_protection_, true);

    pnh.param<double>("invalid_pose_timeout", invalid_pose_timeout_, 0.05);
    pnh.param<int>("invalid_pose_min_count", invalid_pose_min_count_, 3);
    if (!std::isfinite(invalid_pose_timeout_) || invalid_pose_timeout_ <= 0.0 ||
        invalid_pose_min_count_ < 2) {
      ROS_FATAL("Invalid continuous-invalid-pose monitoring parameters");
      ros::shutdown();
      return;
    }

    if (!std::isfinite(max_flight_speed_) || !std::isfinite(max_angular_speed_deg_) ||
        !std::isfinite(vision_pose_timeout_sec_) || max_flight_speed_ <= 0.0 ||
        max_angular_speed_deg_ <= 0.0 || vision_pose_timeout_sec_ <= 0.0)
    {
      ROS_FATAL("Invalid monitoring parameters: max_flight_speed=%.3f, "
                "max_angular_speed=%.3f deg/s, vision_pose_timeout=%.3f s",
                max_flight_speed_, max_angular_speed_deg_, vision_pose_timeout_sec_);
      ros::shutdown();
      return;
    }
    max_angular_speed_rad_ = max_angular_speed_deg_ * std::acos(-1.0) / 180.0;

    safety_.reset(new SafetyMonitor(vision_pose_timeout_sec_, invalid_pose_timeout_,
                                    static_cast<unsigned>(invalid_pose_min_count_)));
    max_message_age_ = vision_pose_timeout_sec_;
    max_linear_speed_ = max_flight_speed_ * 2.0;
    loadExtrinsic(pnh);

    vision_pub_ = nh.advertise<geometry_msgs::PoseStamped>(output_topic_, 1);
    diagnostics_pub_ = nh.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 1);
    set_mode_client_ = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    odom_sub_ = nh.subscribe(input_topic_, 1, &LioToMavros::odomCallback, this,
                             ros::TransportHints().tcpNoDelay());
    watchdog_nh_.reset(new ros::NodeHandle(nh));
    watchdog_nh_->setCallbackQueue(&watchdog_queue_);
    watchdog_timer_ = watchdog_nh_->createSteadyTimer(
        ros::WallDuration(0.01), &LioToMavros::watchdogCallback, this);
    watchdog_spinner_.reset(new ros::AsyncSpinner(1, &watchdog_queue_));
    watchdog_spinner_->start();
    diagnostics_timer_ = nh.createSteadyTimer(ros::WallDuration(diagnostic_period_),
                                            &LioToMavros::diagnosticsCallback, this);

    ROS_INFO("Subscribed to LIO odometry: %s", input_topic_.c_str());
    ROS_INFO("Publishing vision pose to: %s", output_topic_.c_str());
  }

  ~LioToMavros()
  {
    watchdog_timer_.stop();
    if (watchdog_spinner_) watchdog_spinner_->stop();
  }

private:
  static double steadyNow() { return ros::SteadyTime::now().toSec(); }

  static const char* faultName(SafetyMonitor::Fault fault)
  {
    switch (fault) {
      case SafetyMonitor::Fault::Timeout: return "measurement timeout";
      case SafetyMonitor::Fault::InvalidSequence: return "continuous invalid poses";
      default: return "none";
    }
  }

  void watchdogCallback(const ros::SteadyTimerEvent&) { evaluateSafety(); }

  void evaluateSafety()
  {
    // Only watchdog callbacks own the request state. Diagnostics take a copy
    // under this separate mutex; odometry never takes this mutex.
    std::lock_guard<std::mutex> lock(land_state_mutex_);
    pollLandRequest();
    const auto fault = safety_->fault(steadyNow());
    if (fault != SafetyMonitor::Fault::None)
      triggerAutoLand(faultName(fault));
  }

  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
  {
    ++received_count_;
    const ros::SteadyTime receive_time = ros::SteadyTime::now();
    receive_times_.push_back(receive_time);
    while (!receive_times_.empty() &&
           (receive_time - receive_times_.front()).toSec() > frequency_window_)
    {
      receive_times_.pop_front();
    }
    last_receive_wall_ = receive_time;
    updateFrequencyEstimate();

    const ros::Time now = ros::Time::now();
    if (!ros::Time::isValid() || now.isZero()) {
      ++dropped_timestamp_count_;
      return;
    }
    if (msg->header.stamp.isZero())
    {
      ++dropped_timestamp_count_;
      ROS_WARN_THROTTLE(1.0, "Received odometry with zero timestamp, skip this message.");
      return;
    }
    if (last_stamp_valid_ && msg->header.stamp < last_stamp_)
    {
      ++dropped_timestamp_count_;
      ROS_WARN_THROTTLE(1.0, "Received out-of-order odometry timestamp, skip this message.");
      return;
    }
    if (now.isValid() && !now.isZero())
    {
      const double timestamp_error = (msg->header.stamp - now).toSec();
      if (timestamp_error > max_future_offset_)
      {
        ++dropped_timestamp_count_;
        ROS_WARN_THROTTLE(1.0, "Odometry timestamp is %.3f s in the future, skip this message.",
                          timestamp_error);
        return;
      }
      last_delay_sec_ = (now - msg->header.stamp).toSec();
      if (last_delay_sec_ >= std::min(max_message_age_, vision_pose_timeout_sec_))
      {
        ++dropped_timestamp_count_;
        ROS_WARN_THROTTLE(1.0, "Odometry is %.3f s old, skip this message.", last_delay_sec_);
        return;
      }
    }

    if (!std::isfinite(msg->pose.pose.position.x) ||
        !std::isfinite(msg->pose.pose.position.y) ||
        !std::isfinite(msg->pose.pose.position.z))
    {
      ++dropped_invalid_count_;
      safety_->reject(steadyNow());
      ROS_WARN_THROTTLE(1.0, "Received non-finite odometry position, skip this message.");
      return;
    }
    if (!std::isfinite(msg->pose.pose.orientation.w) ||
        !std::isfinite(msg->pose.pose.orientation.x) ||
        !std::isfinite(msg->pose.pose.orientation.y) ||
        !std::isfinite(msg->pose.pose.orientation.z))
    {
      ++dropped_invalid_count_;
      safety_->reject(steadyNow());
      ROS_WARN_THROTTLE(1.0, "Received non-finite odometry quaternion, skip this message.");
      return;
    }

    const double position_variance = std::max(
        msg->pose.covariance[0],
        std::max(msg->pose.covariance[7], msg->pose.covariance[14]));
    const double orientation_variance = std::max(
        msg->pose.covariance[21],
        std::max(msg->pose.covariance[28], msg->pose.covariance[35]));
    const bool covariance_is_valid =
        std::isfinite(msg->pose.covariance[0]) &&
        std::isfinite(msg->pose.covariance[7]) &&
        std::isfinite(msg->pose.covariance[14]) &&
        std::isfinite(msg->pose.covariance[21]) &&
        std::isfinite(msg->pose.covariance[28]) &&
        std::isfinite(msg->pose.covariance[35]) &&
        msg->pose.covariance[0] >= 0.0 && msg->pose.covariance[7] >= 0.0 &&
        msg->pose.covariance[14] >= 0.0 && msg->pose.covariance[21] >= 0.0 &&
        msg->pose.covariance[28] >= 0.0 && msg->pose.covariance[35] >= 0.0;
    if (!covariance_is_valid ||
        (max_position_variance_ > 0.0 && position_variance > max_position_variance_) ||
        (max_orientation_variance_ > 0.0 && orientation_variance > max_orientation_variance_))
    {
      ++dropped_drift_count_;
      safety_->reject(steadyNow());
      ROS_WARN_THROTTLE(1.0, "Odometry covariance indicates poor localization, skip this message.");
      return;
    }

    Eigen::Quaterniond q_world_lio_body(msg->pose.pose.orientation.w,
                                            msg->pose.pose.orientation.x,
                                            msg->pose.pose.orientation.y,
                                            msg->pose.pose.orientation.z);
    if (!std::isfinite(q_world_lio_body.norm()) || q_world_lio_body.norm() < 1e-6)
    {
      ++dropped_invalid_count_;
      safety_->reject(steadyNow());
      ROS_WARN_THROTTLE(1.0, "Received invalid odometry quaternion, skip this message.");
      return;
    }
    q_world_lio_body.normalize();

    Eigen::Isometry3d t_world_lio_body = Eigen::Isometry3d::Identity();
    t_world_lio_body.linear() = q_world_lio_body.toRotationMatrix();
    t_world_lio_body.translation() = Eigen::Vector3d(msg->pose.pose.position.x,
                                                         msg->pose.pose.position.y,
                                                         msg->pose.pose.position.z);

    const Eigen::Isometry3d t_world_fcu =
        t_world_lio_body * t_fcu_lio_body_.inverse();
    Eigen::Quaterniond q_world_fcu(t_world_fcu.rotation());
    q_world_fcu.normalize();

    if (!std::isfinite(t_world_fcu.translation().x()) ||
        !std::isfinite(t_world_fcu.translation().y()) ||
        !std::isfinite(t_world_fcu.translation().z()) ||
        !std::isfinite(q_world_fcu.x()) || !std::isfinite(q_world_fcu.y()) ||
        !std::isfinite(q_world_fcu.z()) || !std::isfinite(q_world_fcu.w()))
    {
      ++dropped_invalid_count_;
      safety_->reject(steadyNow());
      ROS_WARN_THROTTLE(1.0, "Extrinsic transform produced non-finite pose, skip this message.");
      return;
    }

    if (last_pose_valid_)
    {
      const double dt = (msg->header.stamp - last_pose_stamp_).toSec();
      if (dt <= 0.0)
      {
        ++dropped_timestamp_count_;
        ROS_WARN_THROTTLE(1.0, "Odometry timestamp did not advance, skip this message.");
        return;
      }

      const Eigen::Vector3d position_delta =
          t_world_fcu.translation() - last_position_;
      const double position_jump = position_delta.norm();
      const double linear_speed = position_jump / dt;
      const Eigen::Quaterniond delta_orientation = last_orientation_.inverse() * q_world_fcu;
      const double angle = Eigen::AngleAxisd(delta_orientation).angle();
      const double angular_speed = angle / dt;
      if ((max_position_jump_ > 0.0 && position_jump > std::max(max_position_jump_, max_linear_speed_ * dt)) ||
          (max_linear_speed_ > 0.0 && linear_speed > max_linear_speed_) ||
          (max_angular_speed_rad_ > 0.0 && angular_speed > max_angular_speed_rad_))
      {
        ++dropped_drift_count_;
        safety_->reject(steadyNow());
        ROS_WARN_THROTTLE(1.0,
                          "Odometry motion is implausible (jump=%.3f m, speed=%.3f m/s, "
                          "angular_speed=%.3f deg/s), skip this message.",
                          position_jump, linear_speed, angular_speed * 180.0 / std::acos(-1.0));
        return;
      }
    }

    const ros::Time publish_now = ros::Time::now();
    const double source_age = (publish_now - msg->header.stamp).toSec();
    if (!ros::Time::isValid() || publish_now.isZero() ||
        source_age < -max_future_offset_ ||
        source_age >= std::min(max_message_age_, vision_pose_timeout_sec_) ||
        !safety_->accept(steadyNow(), std::max(0.0, source_age))) {
      ++dropped_timestamp_count_;
      return;
    }

    // Only advance the timestamp watermark after the complete pose is valid.
    last_stamp_ = msg->header.stamp;
    last_stamp_valid_ = true;
    last_pose_stamp_ = msg->header.stamp;
    last_position_ = t_world_fcu.translation();
    last_orientation_ = q_world_fcu;
    last_pose_valid_ = true;

    geometry_msgs::PoseStamped vision_pose;
    vision_pose.header.stamp = msg->header.stamp;
    vision_pose.header.frame_id = frame_id_;

    vision_pose.pose.position.x = t_world_fcu.translation().x();
    vision_pose.pose.position.y = t_world_fcu.translation().y();
    vision_pose.pose.position.z = t_world_fcu.translation().z();

    vision_pose.pose.orientation.x = q_world_fcu.x();
    vision_pose.pose.orientation.y = q_world_fcu.y();
    vision_pose.pose.orientation.z = q_world_fcu.z();
    vision_pose.pose.orientation.w = q_world_fcu.w();

    vision_pub_.publish(vision_pose);
    last_publish_wall_ = ros::SteadyTime::now();
    ++published_count_;
  }

  void triggerAutoLand(const char* reason)
  {
    if (!enable_vision_pose_timeout_protection_ || auto_land_triggered_ ||
        land_request_.valid() ||
        (!last_land_attempt_.isZero() &&
         (ros::SteadyTime::now() - last_land_attempt_).toSec() < 0.5))
      return;

    // Copy the client; the worker never accesses node state.
    last_land_attempt_ = ros::SteadyTime::now();
    ros::ServiceClient client = set_mode_client_;
    land_request_ = std::async(std::launch::async, [client]() mutable {
      mavros_msgs::SetMode request;
      request.request.base_mode = 0;
      request.request.custom_mode = "AUTO.LAND";
      if (!client.call(request)) return 0;
      return request.response.mode_sent ? 1 : 2;
    });
    auto_land_triggered_ = true;
    auto_land_result_ = "AUTO.LAND request in progress";
    ROS_ERROR("%s; requesting AUTO.LAND asynchronously.", reason);
  }

  void pollLandRequest()
  {
    if (!land_request_.valid() ||
        land_request_.wait_for(std::chrono::seconds(0)) != std::future_status::ready)
      return;
    try {
      const int result = land_request_.get();
      auto_land_result_ = result == 1 ? "AUTO.LAND request sent (mode not confirmed)"
                                    : "AUTO.LAND request failed";
      if (result != 1) {
        auto_land_triggered_ = false;
        ROS_ERROR("AUTO.LAND request failed; will retry while output is stale.");
      }
    } catch (const std::exception& error) {
      auto_land_triggered_ = false;
      auto_land_result_ = error.what();
      ROS_ERROR("AUTO.LAND request failed: %s", error.what());
    }
  }

  void updateFrequencyEstimate()
  {
    if (frequency_learned_ || receive_times_.size() < 2)
    {
      return;
    }

    const double window = (receive_times_.back() - receive_times_.front()).toSec();
    if (window < frequency_learning_window_)
    {
      return;
    }

    nominal_input_frequency_ =
        static_cast<double>(receive_times_.size() - 1) / window;
    if (nominal_input_frequency_ <= 0.0 || !std::isfinite(nominal_input_frequency_))
    {
      return;
    }

    min_input_frequency_ = nominal_input_frequency_ * 0.5;
    max_message_age_ = std::min(vision_pose_timeout_sec_,
                                std::max(0.2, 3.0 / nominal_input_frequency_));
    max_linear_speed_ = max_flight_speed_ * 2.0;
    max_position_jump_ = max_linear_speed_ / nominal_input_frequency_ * 2.0;
    frequency_learned_ = true;

    ROS_INFO("Learned input frequency: %.3f Hz; derived thresholds: "
             "max_message_age=%.3f s, min_input_frequency=%.3f Hz, "
             "max_position_jump=%.3f m",
             nominal_input_frequency_, max_message_age_, min_input_frequency_,
             max_position_jump_);
  }

  void diagnosticsCallback(const ros::SteadyTimerEvent&)
  {
    const ros::SteadyTime now_wall = ros::SteadyTime::now();
    const double receive_age = received_count_ == 0
                                   ? 0.0
                                   : (now_wall - last_receive_wall_).toSec();
    const double publish_age = published_count_ == 0
                                   ? 0.0
                                   : (now_wall - last_publish_wall_).toSec();
    const double frequency_window = receive_times_.size() >= 2
                                      ? (receive_times_.back() - receive_times_.front()).toSec()
                                      : 0.0;
    const double frequency = receive_times_.size() >= 2 && frequency_window > 1e-6
                               ? static_cast<double>(receive_times_.size() - 1) / frequency_window
                               : 0.0;

    const auto current_fault = safety_->fault(steadyNow());
    const bool vision_pose_is_stale = current_fault != SafetyMonitor::Fault::None;
    bool land_triggered;
    std::string land_result;
    {
      std::lock_guard<std::mutex> lock(land_state_mutex_);
      land_triggered = auto_land_triggered_;
      land_result = auto_land_result_;
    }

    diagnostic_msgs::DiagnosticArray array;
    array.header.stamp = ros::Time::now();
    diagnostic_msgs::DiagnosticStatus status;
    status.name = "odom_to_mavros/input";
    status.hardware_id = "odom_to_mavros";
    status.level = diagnostic_msgs::DiagnosticStatus::OK;
    status.message = "Operating normally";

    if (vision_pose_is_stale)
    {
      status.level = diagnostic_msgs::DiagnosticStatus::ERROR;
      status.message = faultName(current_fault);
    }
    else if (received_count_ == 0)
    {
      status.level = diagnostic_msgs::DiagnosticStatus::WARN;
      status.message = "Waiting for odometry";
    }
    else if (receive_age > max_message_age_)
    {
      status.level = diagnostic_msgs::DiagnosticStatus::ERROR;
      status.message = "Odometry input is stale";
    }
    else if (frequency_learned_ && frequency < min_input_frequency_)
    {
      status.level = diagnostic_msgs::DiagnosticStatus::WARN;
      status.message = "Odometry input frequency is low";
    }
    else if (dropped_drift_count_ > 0)
    {
      status.level = diagnostic_msgs::DiagnosticStatus::WARN;
      status.message = "Odometry quality or motion check rejected messages";
    }
    else if (dropped_invalid_count_ > 0 || dropped_timestamp_count_ > 0)
    {
      status.level = diagnostic_msgs::DiagnosticStatus::WARN;
      status.message = "Some odometry messages were rejected";
    }

    addDiagnosticValue(status, "input_topic", input_topic_);
    addDiagnosticValue(status, "received_count", received_count_);
    addDiagnosticValue(status, "published_count", published_count_);
    addDiagnosticValue(status, "invalid_drop_count", dropped_invalid_count_);
    addDiagnosticValue(status, "timestamp_drop_count", dropped_timestamp_count_);
    addDiagnosticValue(status, "drift_check_drop_count", dropped_drift_count_);
    addDiagnosticValue(status, "frequency_hz", frequency);
    addDiagnosticValue(status, "nominal_frequency_hz", nominal_input_frequency_);
    addDiagnosticValue(status, "last_receive_age_sec", receive_age);
    addDiagnosticValue(status, "last_publish_age_sec", publish_age);
    addDiagnosticValue(status, "last_message_delay_sec", last_delay_sec_);
    addDiagnosticValue(status, "vision_pose_timeout_sec", vision_pose_timeout_sec_);
    addDiagnosticValue(status, "vision_pose_timeout_protection_enabled",
                       enable_vision_pose_timeout_protection_ ? "true" : "false");
    addDiagnosticValue(status, "auto_land_triggered", land_triggered ? "true" : "false");
    addDiagnosticValue(status, "auto_land_result", land_result);
    array.status.push_back(status);
    diagnostics_pub_.publish(array);
  }

  static void addDiagnosticValue(diagnostic_msgs::DiagnosticStatus& status,
                                 const std::string& key, const std::string& value)
  {
    diagnostic_msgs::KeyValue item;
    item.key = key;
    item.value = value;
    status.values.push_back(item);
  }

  static void addDiagnosticValue(diagnostic_msgs::DiagnosticStatus& status,
                                 const std::string& key, double value)
  {
    addDiagnosticValue(status, key, std::to_string(value));
  }

  static void addDiagnosticValue(diagnostic_msgs::DiagnosticStatus& status,
                                 const std::string& key, uint64_t value)
  {
    addDiagnosticValue(status, key, std::to_string(value));
  }

  void loadExtrinsic(const ros::NodeHandle& pnh)
  {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    std::string rpy_unit = "rad";

    pnh.param<double>("extrinsic/translation/x", x, 0.0);
    pnh.param<double>("extrinsic/translation/y", y, 0.0);
    pnh.param<double>("extrinsic/translation/z", z, 0.0);
    pnh.param<double>("extrinsic/rotation/roll", roll, 0.0);
    pnh.param<double>("extrinsic/rotation/pitch", pitch, 0.0);
    pnh.param<double>("extrinsic/rotation/yaw", yaw, 0.0);
    pnh.param<std::string>("extrinsic/rotation/unit", rpy_unit, "rad");

    if (rpy_unit == "deg" || rpy_unit == "degree" || rpy_unit == "degrees")
    {
      const double kDegToRad = std::acos(-1.0) / 180.0;
      roll *= kDegToRad;
      pitch *= kDegToRad;
      yaw *= kDegToRad;
    }
    else if (rpy_unit != "rad" && rpy_unit != "radian" && rpy_unit != "radians")
    {
      ROS_WARN("Unknown extrinsic rotation unit '%s', use radians.", rpy_unit.c_str());
    }

    const Eigen::AngleAxisd roll_angle(roll, Eigen::Vector3d::UnitX());
    const Eigen::AngleAxisd pitch_angle(pitch, Eigen::Vector3d::UnitY());
    const Eigen::AngleAxisd yaw_angle(yaw, Eigen::Vector3d::UnitZ());
    const Eigen::Quaterniond q_fcu_lio_body = yaw_angle * pitch_angle * roll_angle;

    t_fcu_lio_body_ = Eigen::Isometry3d::Identity();
    t_fcu_lio_body_.linear() = q_fcu_lio_body.toRotationMatrix();
    t_fcu_lio_body_.translation() = Eigen::Vector3d(x, y, z);

    ROS_INFO("Loaded T_fcu_lio_body: xyz = [%.4f, %.4f, %.4f], "
             "rpy(rad) = [%.4f, %.4f, %.4f]",
             x, y, z, roll, pitch, yaw);
  }

  std::unique_ptr<SafetyMonitor> safety_;
  ros::CallbackQueue watchdog_queue_;
  std::unique_ptr<ros::NodeHandle> watchdog_nh_;
  std::unique_ptr<ros::AsyncSpinner> watchdog_spinner_;
  ros::SteadyTimer watchdog_timer_;
  std::mutex land_state_mutex_;
  double invalid_pose_timeout_ = 0.05;
  int invalid_pose_min_count_ = 3;
  std::future<int> land_request_;
  ros::SteadyTime last_land_attempt_;
  ros::Subscriber odom_sub_;
  ros::Publisher vision_pub_;
  ros::Publisher diagnostics_pub_;
  ros::SteadyTimer diagnostics_timer_;
  ros::ServiceClient set_mode_client_;
  Eigen::Isometry3d t_fcu_lio_body_ = Eigen::Isometry3d::Identity();
  std::string input_topic_;
  std::string output_topic_;
  std::string frame_id_;
  double max_flight_speed_ = 5.0;
  // Always capped by the measurement timeout, including during startup.
  double max_message_age_ = 0.2;
  double max_future_offset_ = 0.1;
  double min_input_frequency_ = 5.0;
  double diagnostic_period_ = 1.0;
  double max_position_variance_ = 4.0;
  double max_orientation_variance_ = 1.0;
  double max_position_jump_ = 2.0;
  double max_linear_speed_ = 10.0;
  double max_angular_speed_deg_ = 180.0;
  double max_angular_speed_rad_ = std::acos(-1.0);
  double vision_pose_timeout_sec_ = 0.2;
  bool enable_vision_pose_timeout_protection_ = true;
  const double frequency_window_ = 2.0;
  const double frequency_learning_window_ = 1.0;
  ros::SteadyTime last_receive_wall_;
  ros::SteadyTime last_publish_wall_;
  std::deque<ros::SteadyTime> receive_times_;
  double nominal_input_frequency_ = 0.0;
  bool frequency_learned_ = false;
  ros::Time last_stamp_;
  bool last_stamp_valid_ = false;
  ros::Time last_pose_stamp_;
  bool last_pose_valid_ = false;
  Eigen::Vector3d last_position_ = Eigen::Vector3d::Zero();
  Eigen::Quaterniond last_orientation_ = Eigen::Quaterniond::Identity();
  double last_delay_sec_ = 0.0;
  uint64_t received_count_ = 0;
  uint64_t published_count_ = 0;
  uint64_t dropped_invalid_count_ = 0;
  uint64_t dropped_timestamp_count_ = 0;
  uint64_t dropped_drift_count_ = 0;
  bool auto_land_triggered_ = false;
  std::string auto_land_result_ = "not requested";
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "odom_to_mavros");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  LioToMavros node(nh, pnh);
  ros::spin();
  return 0;
}

