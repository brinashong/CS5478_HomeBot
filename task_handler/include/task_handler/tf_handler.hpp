#ifndef TF_HANDLER_HPP
#define TF_HANDLER_HPP

#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>

#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>

class TFHandler
{
public:
  TFHandler(ros::NodeHandle &nh, ros::NodeHandle &pnh);
  ~TFHandler();

  void gazeboCallback(const gazebo_msgs::ModelStates::ConstPtr &msg);
  void publishTF();

private:
  ros::Subscriber gz_subscriber_;
  ros::Publisher amcl_pose_publisher_;

  tf2_ros::TransformBroadcaster tf_pub_;

  geometry_msgs::PoseWithCovarianceStamped amcl_pose_;
  geometry_msgs::TransformStamped odom_to_baselink_tf_;

  std::string robotname_;
  bool pose_updated_;
};

#endif /* TF_HANDLER_HPP */

