#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit_msgs/JointConstraint.h"
#include "ros/node_handle.h"
#include "task_handler/moveit_client.hpp"
#include <memory>
#include <thread>

int main (int argc, char *argv[])
{
  ros::init(argc, argv, "moveit_client_node");
  ros::AsyncSpinner spinner(1);
  ros::NodeHandle nh_;
  spinner.start();

  static const std::string arm_planning_group = "stretch_arm";
  auto arm_control_interface =
    std::make_shared<moveit::planning_interface::MoveGroupInterface>(arm_planning_group);

  moveit_control::MoveItClient control_arm(
      nh_,
      arm_control_interface,
      arm_planning_group);

  ROS_INFO_STREAM("MOVING TO EXTENDED POSITION");
  control_arm.goPreset("extended");
  control_arm.addBox({1, 1, 0.3}, "test_box", "map");
  ROS_INFO_STREAM("DONE MOVING TO EXTENDED POSITION!!!!!");
  std::this_thread::sleep_for(std::chrono::seconds(1));

  ROS_INFO_STREAM("MOVING TO HOME POSITION");
  control_arm.movePose({-1.0, -1.0, 0.0, 0.0, 0.0, 0.0});
  // control_arm.moveRandomValidPose();
  ROS_INFO_STREAM("DONE MOVING TO HOME POSITION!!!!!");
  // control_arm.moveZWithCurrentPoseDefaultConstraints(0.4);

  control_arm.printUsefulInfo();

  return 0;
}
