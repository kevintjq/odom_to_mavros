#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

class FastlioToMavros
{
public:
  FastlioToMavros(ros::NodeHandle& nh, ros::NodeHandle& pnh)
  {
    pnh.param<std::string>("input_topic", input_topic_, "/Odometry");
    pnh.param<std::string>("output_topic", output_topic_, "/mavros/vision_pose/pose");
    pnh.param<std::string>("frame_id", frame_id_, "world");

    vision_pub_ = nh.advertise<geometry_msgs::PoseStamped>(output_topic_, 10);
    odom_sub_ = nh.subscribe(input_topic_, 100, &FastlioToMavros::odomCallback, this);

    ROS_INFO("Subscribed to LIO odometry: %s", input_topic_.c_str());
    ROS_INFO("Publishing vision pose to: %s", output_topic_.c_str());
  }

private:
  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
  {
    geometry_msgs::PoseStamped vision_pose;
    vision_pose.header.stamp = msg->header.stamp;
    vision_pose.header.frame_id = frame_id_;

    vision_pose.pose.position.x = msg->pose.pose.position.x;
    vision_pose.pose.position.y = msg->pose.pose.position.y;
    vision_pose.pose.position.z = msg->pose.pose.position.z;

    vision_pose.pose.orientation.x = 0.0;
    vision_pose.pose.orientation.y = 0.0;
    vision_pose.pose.orientation.z = 0.0;
    vision_pose.pose.orientation.w = 1.0;

    vision_pub_.publish(vision_pose);
  }

  ros::Subscriber odom_sub_;
  ros::Publisher vision_pub_;
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
