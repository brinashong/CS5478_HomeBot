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
  static const std::string gripper_planning_group = "stretch_gripper";
  auto arm_control_interface =
    std::make_shared<moveit::planning_interface::MoveGroupInterface>(arm_planning_group);
  auto gripper_control_interface =
    std::make_shared<moveit::planning_interface::MoveGroupInterface>(gripper_planning_group);

  moveit_control::MoveItClient control_arm(
      nh_,
      arm_control_interface,
      gripper_control_interface,
      arm_planning_group);

  ROS_INFO_STREAM("MOVING TO HOME");
  control_arm.goPreset("home");
  ROS_INFO_STREAM("DONE MOVING TO HOME!!!!!");
  std::this_thread::sleep_for(std::chrono::seconds(1));

  ROS_INFO_STREAM("MOVING TO HOVER");
  control_arm.goPreset("hover");
  ROS_INFO_STREAM("DONE MOVING TO HOVER!!!!!");
  std::this_thread::sleep_for(std::chrono::seconds(1));

  ROS_INFO_STREAM("MOVING TO APPROACH");
  control_arm.goPreset("approach");
  ROS_INFO_STREAM("DONE MOVING TO APPROACH!!!!!");
  std::this_thread::sleep_for(std::chrono::seconds(1));

  ROS_INFO_STREAM("MOVING TO GRIPPER TO SODA CAN PRESET");
  control_arm.goPresetGripper("soda_can");
  ROS_INFO_STREAM("DONE MOVING TO GRIPPER TO SODA CAN PRESET!!!!!");

  ROS_WARN_STREAM("ATTACHING CAN PEPSI");
  control_arm.attachGazeboModel("Coke", "link");
  ROS_WARN_STREAM("FINISHED ATTACHING CAN PEPSI");

  // ROS_WARN_STREAM("DETACHING CAN PEPSI");
  // control_arm.detachGazeboModel("Coke", "link");
  // ROS_WARN_STREAM("FINISHED DETACHING CAN PEPSI");

  control_arm.printUsefulInfo();

  return 0;
}
