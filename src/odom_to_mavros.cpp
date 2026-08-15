#include <Eigen/Geometry>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

#include <cmath>
#include <string>

class FastlioToMavros
{
public:
  FastlioToMavros(ros::NodeHandle& nh, ros::NodeHandle& pnh)
  {
    pnh.param<std::string>("input_topic", input_topic_, "/Odometry");
    pnh.param<std::string>("output_topic", output_topic_, "/mavros/vision_pose/pose");
    pnh.param<std::string>("frame_id", frame_id_, "world");

    loadExtrinsic(pnh);

    vision_pub_ = nh.advertise<geometry_msgs::PoseStamped>(output_topic_, 10);
    odom_sub_ = nh.subscribe(input_topic_, 100, &FastlioToMavros::odomCallback, this);

    ROS_INFO("Subscribed to LIO odometry: %s", input_topic_.c_str());
    ROS_INFO("Publishing vision pose to: %s", output_topic_.c_str());
  }

private:
  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
  {
    if (!std::isfinite(msg->pose.pose.orientation.w) ||
        !std::isfinite(msg->pose.pose.orientation.x) ||
        !std::isfinite(msg->pose.pose.orientation.y) ||
        !std::isfinite(msg->pose.pose.orientation.z))
    {
      ROS_WARN_THROTTLE(1.0, "Received non-finite odometry quaternion, skip this message.");
      return;
    }

    Eigen::Quaterniond q_world_lidar(msg->pose.pose.orientation.w,
                                     msg->pose.pose.orientation.x,
                                     msg->pose.pose.orientation.y,
                                     msg->pose.pose.orientation.z);
    if (q_world_lidar.norm() < 1e-6)
    {
      ROS_WARN_THROTTLE(1.0, "Received invalid odometry quaternion, skip this message.");
      return;
    }
    q_world_lidar.normalize();

    Eigen::Isometry3d t_world_lidar = Eigen::Isometry3d::Identity();
    t_world_lidar.linear() = q_world_lidar.toRotationMatrix();
    t_world_lidar.translation() = Eigen::Vector3d(msg->pose.pose.position.x,
                                                  msg->pose.pose.position.y,
                                                  msg->pose.pose.position.z);

    const Eigen::Isometry3d t_world_body = t_world_lidar * t_body_lidar_.inverse();
    Eigen::Quaterniond q_world_body(t_world_body.rotation());
    q_world_body.normalize();

    geometry_msgs::PoseStamped vision_pose;
    vision_pose.header.stamp = msg->header.stamp;
    vision_pose.header.frame_id = frame_id_;

    vision_pose.pose.position.x = t_world_body.translation().x();
    vision_pose.pose.position.y = t_world_body.translation().y();
    vision_pose.pose.position.z = t_world_body.translation().z();

    vision_pose.pose.orientation.x = q_world_body.x();
    vision_pose.pose.orientation.y = q_world_body.y();
    vision_pose.pose.orientation.z = q_world_body.z();
    vision_pose.pose.orientation.w = q_world_body.w();

    vision_pub_.publish(vision_pose);
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
    const Eigen::Quaterniond q_body_lidar = yaw_angle * pitch_angle * roll_angle;

    t_body_lidar_ = Eigen::Isometry3d::Identity();
    t_body_lidar_.linear() = q_body_lidar.toRotationMatrix();
    t_body_lidar_.translation() = Eigen::Vector3d(x, y, z);

    ROS_INFO("Loaded T_body_lidar: xyz = [%.4f, %.4f, %.4f], rpy(rad) = [%.4f, %.4f, %.4f]",
             x, y, z, roll, pitch, yaw);
  }

  ros::Subscriber odom_sub_;
  ros::Publisher vision_pub_;
  Eigen::Isometry3d t_body_lidar_ = Eigen::Isometry3d::Identity();
  std::string input_topic_;
  std::string output_topic_;
  std::string frame_id_;
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
