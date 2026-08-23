#include <Eigen/Geometry>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>
#include <ros/ros.h>

#include <cmath>
#include <cstdint>
#include <deque>
#include <string>

class FastlioToMavros
{
public:
  FastlioToMavros(ros::NodeHandle& nh, ros::NodeHandle& pnh)
  {
    pnh.param<std::string>("input_topic", input_topic_, "/Odometry");
    pnh.param<std::string>("output_topic", output_topic_, "/mavros/vision_pose/pose");
    pnh.param<std::string>("frame_id", frame_id_, "world");
    pnh.param<double>("max_message_age", max_message_age_, 0.5);
    pnh.param<double>("max_future_offset", max_future_offset_, 0.1);
    pnh.param<double>("min_input_frequency", min_input_frequency_, 5.0);
    pnh.param<double>("diagnostic_period", diagnostic_period_, 1.0);

    if (max_message_age_ <= 0.0 || max_future_offset_ < 0.0 ||
        min_input_frequency_ < 0.0 || diagnostic_period_ <= 0.0)
    {
      ROS_FATAL("Invalid monitoring parameters: max_message_age=%.3f, "
                "max_future_offset=%.3f, min_input_frequency=%.3f, "
                "diagnostic_period=%.3f",
                max_message_age_, max_future_offset_, min_input_frequency_, diagnostic_period_);
      ros::shutdown();
      return;
    }

    loadExtrinsic(pnh);

    vision_pub_ = nh.advertise<geometry_msgs::PoseStamped>(output_topic_, 10);
    diagnostics_pub_ = nh.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 1);
    odom_sub_ = nh.subscribe(input_topic_, 100, &FastlioToMavros::odomCallback, this);
    diagnostics_timer_ = nh.createTimer(ros::Duration(diagnostic_period_),
                                        &FastlioToMavros::diagnosticsCallback, this);

    ROS_INFO("Subscribed to LIO odometry: %s", input_topic_.c_str());
    ROS_INFO("Publishing vision pose to: %s", output_topic_.c_str());
  }

private:
  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
  {
    ++received_count_;
    const ros::WallTime receive_time = ros::WallTime::now();
    receive_times_.push_back(receive_time);
    while (!receive_times_.empty() &&
           (receive_time - receive_times_.front()).toSec() > frequency_window_)
    {
      receive_times_.pop_front();
    }
    last_receive_wall_ = receive_time;

    const ros::Time now = ros::Time::now();
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
      if (last_delay_sec_ > max_message_age_)
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
      ROS_WARN_THROTTLE(1.0, "Received non-finite odometry position, skip this message.");
      return;
    }
    if (!std::isfinite(msg->pose.pose.orientation.w) ||
        !std::isfinite(msg->pose.pose.orientation.x) ||
        !std::isfinite(msg->pose.pose.orientation.y) ||
        !std::isfinite(msg->pose.pose.orientation.z))
    {
      ++dropped_invalid_count_;
      ROS_WARN_THROTTLE(1.0, "Received non-finite odometry quaternion, skip this message.");
      return;
    }

    Eigen::Quaterniond q_world_fastlio_body(msg->pose.pose.orientation.w,
                                            msg->pose.pose.orientation.x,
                                            msg->pose.pose.orientation.y,
                                            msg->pose.pose.orientation.z);
    if (!std::isfinite(q_world_fastlio_body.norm()) || q_world_fastlio_body.norm() < 1e-6)
    {
      ++dropped_invalid_count_;
      ROS_WARN_THROTTLE(1.0, "Received invalid odometry quaternion, skip this message.");
      return;
    }
    q_world_fastlio_body.normalize();

    Eigen::Isometry3d t_world_fastlio_body = Eigen::Isometry3d::Identity();
    t_world_fastlio_body.linear() = q_world_fastlio_body.toRotationMatrix();
    t_world_fastlio_body.translation() = Eigen::Vector3d(msg->pose.pose.position.x,
                                                         msg->pose.pose.position.y,
                                                         msg->pose.pose.position.z);

    const Eigen::Isometry3d t_world_fcu =
        t_world_fastlio_body * t_fcu_fastlio_body_.inverse();
    Eigen::Quaterniond q_world_fcu(t_world_fcu.rotation());
    q_world_fcu.normalize();

    if (!std::isfinite(t_world_fcu.translation().x()) ||
        !std::isfinite(t_world_fcu.translation().y()) ||
        !std::isfinite(t_world_fcu.translation().z()) ||
        !std::isfinite(q_world_fcu.x()) || !std::isfinite(q_world_fcu.y()) ||
        !std::isfinite(q_world_fcu.z()) || !std::isfinite(q_world_fcu.w()))
    {
      ++dropped_invalid_count_;
      ROS_WARN_THROTTLE(1.0, "Extrinsic transform produced non-finite pose, skip this message.");
      return;
    }

    // Only advance the timestamp watermark after the complete pose is valid.
    last_stamp_ = msg->header.stamp;
    last_stamp_valid_ = true;

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
    ++published_count_;
  }

  void diagnosticsCallback(const ros::TimerEvent&)
  {
    const ros::WallTime now_wall = ros::WallTime::now();
    const double receive_age = received_count_ == 0
                                   ? 0.0
                                   : (now_wall - last_receive_wall_).toSec();
    const double frequency_window = receive_times_.size() >= 2
                                      ? (receive_times_.back() - receive_times_.front()).toSec()
                                      : 0.0;
    const double frequency = receive_times_.size() >= 2 && frequency_window > 1e-6
                               ? static_cast<double>(receive_times_.size() - 1) / frequency_window
                               : 0.0;

    diagnostic_msgs::DiagnosticArray array;
    array.header.stamp = ros::Time::now();
    diagnostic_msgs::DiagnosticStatus status;
    status.name = "odom_to_mavros/input";
    status.hardware_id = "odom_to_mavros";
    status.level = diagnostic_msgs::DiagnosticStatus::OK;
    status.message = "Operating normally";

    if (received_count_ == 0)
    {
      status.level = diagnostic_msgs::DiagnosticStatus::WARN;
      status.message = "Waiting for odometry";
    }
    else if (receive_age > max_message_age_)
    {
      status.level = diagnostic_msgs::DiagnosticStatus::ERROR;
      status.message = "Odometry input is stale";
    }
    else if (receive_times_.size() >= 2 && frequency < min_input_frequency_)
    {
      status.level = diagnostic_msgs::DiagnosticStatus::WARN;
      status.message = "Odometry input frequency is low";
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
    addDiagnosticValue(status, "frequency_hz", frequency);
    addDiagnosticValue(status, "last_receive_age_sec", receive_age);
    addDiagnosticValue(status, "last_message_delay_sec", last_delay_sec_);
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
    const Eigen::Quaterniond q_fcu_fastlio_body = yaw_angle * pitch_angle * roll_angle;

    t_fcu_fastlio_body_ = Eigen::Isometry3d::Identity();
    t_fcu_fastlio_body_.linear() = q_fcu_fastlio_body.toRotationMatrix();
    t_fcu_fastlio_body_.translation() = Eigen::Vector3d(x, y, z);

    ROS_INFO("Loaded T_fcu_fastlio_body: xyz = [%.4f, %.4f, %.4f], "
             "rpy(rad) = [%.4f, %.4f, %.4f]",
             x, y, z, roll, pitch, yaw);
  }

  ros::Subscriber odom_sub_;
  ros::Publisher vision_pub_;
  ros::Publisher diagnostics_pub_;
  ros::Timer diagnostics_timer_;
  Eigen::Isometry3d t_fcu_fastlio_body_ = Eigen::Isometry3d::Identity();
  std::string input_topic_;
  std::string output_topic_;
  std::string frame_id_;
  double max_message_age_ = 0.5;
  double max_future_offset_ = 0.1;
  double min_input_frequency_ = 5.0;
  double diagnostic_period_ = 1.0;
  const double frequency_window_ = 2.0;
  ros::WallTime last_receive_wall_;
  std::deque<ros::WallTime> receive_times_;
  ros::Time last_stamp_;
  bool last_stamp_valid_ = false;
  double last_delay_sec_ = 0.0;
  uint64_t received_count_ = 0;
  uint64_t published_count_ = 0;
  uint64_t dropped_invalid_count_ = 0;
  uint64_t dropped_timestamp_count_ = 0;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "odom_to_mavros");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  FastlioToMavros node(nh, pnh);
  ros::spin();
  return 0;
}
