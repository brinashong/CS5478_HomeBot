#include "task_handler/tf_handler.hpp"

TFHandler::TFHandler(ros::NodeHandle &nh, ros::NodeHandle &pnh) : pose_updated_(false)
{
  gz_subscriber_ = nh.subscribe("gazebo/model_states", 1, &TFHandler::gazeboCallback, this);
  amcl_pose_publisher_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose", 1);

  pnh.param<std::string>("robotname", robotname_, "robot");

  odom_to_baselink_tf_.header.frame_id = "odom";
  odom_to_baselink_tf_.child_frame_id = "base_link";
  amcl_pose_.header.frame_id = "odom";
}

TFHandler::~TFHandler() {}

void TFHandler::gazeboCallback(const gazebo_msgs::ModelStates::ConstPtr &msg)
{
  // get robot info
  int robotindex;
  auto it = find(msg->name.begin(), msg->name.end(), robotname_);
  if (it != msg->name.end())
  {
    robotindex = it - msg->name.begin();
    amcl_pose_.pose.pose = msg->pose[robotindex];
    pose_updated_ = true;
  }
  else
  {
    ROS_ERROR_STREAM("Failed to find robot: " << robotname_ << " in gazebo model list");
    return;
  }
}

void TFHandler::publishTF()
{
  if (pose_updated_)
  {
    odom_to_baselink_tf_.header.stamp = ros::Time::now();
    odom_to_baselink_tf_.transform.translation.x = amcl_pose_.pose.pose.position.x;
    odom_to_baselink_tf_.transform.translation.y = amcl_pose_.pose.pose.position.y;
    odom_to_baselink_tf_.transform.translation.z = amcl_pose_.pose.pose.position.z;
    odom_to_baselink_tf_.transform.rotation.x = amcl_pose_.pose.pose.orientation.x;
    odom_to_baselink_tf_.transform.rotation.y = amcl_pose_.pose.pose.orientation.y;
    odom_to_baselink_tf_.transform.rotation.z = amcl_pose_.pose.pose.orientation.z;
    odom_to_baselink_tf_.transform.rotation.w = amcl_pose_.pose.pose.orientation.w;
    tf_pub_.sendTransform(odom_to_baselink_tf_);

    amcl_pose_.header.stamp = ros::Time::now();
    amcl_pose_.pose.covariance = {
        1, 0, 0, 0, 0, 0,
        0, 1, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 1};
    amcl_pose_publisher_.publish(amcl_pose_);

    pose_updated_ = false;
  }
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "tf_handler");
  ros::NodeHandle nh, pnh("~");

  // Initialize the object
  TFHandler TF(nh, pnh);

  double tf_publish_rate;
  pnh.param<double>("tf_publish_rate", tf_publish_rate, 25.0);
  ros::Rate rate(tf_publish_rate);

  // Main loop
  while (ros::ok())
  {
    TF.publishTF();

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}

