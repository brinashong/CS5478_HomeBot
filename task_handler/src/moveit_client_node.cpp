#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit_msgs/JointConstraint.h"
#include "ros/node_handle.h"
#include "task_handler/moveit_client.hpp"
#include <memory>
#include <thread>

int main (int argc, char *argv[])
{
  ros::init(argc, argv, "moveit_client_node");
  ros::AsyncSpinner spinner(3);
  ros::NodeHandle nh_;
  spinner.start();

  static const std::string arm_planning_group = "stretch_arm";
  static const std::string gripper_planning_group = "stretch_gripper";
  static const std::string camera_planning_group = "stretch_head";
  auto arm_control_interface =
    std::make_shared<moveit::planning_interface::MoveGroupInterface>(arm_planning_group);
  auto gripper_control_interface =
    std::make_shared<moveit::planning_interface::MoveGroupInterface>(gripper_planning_group);
  auto camera_control_interface =
    std::make_shared<moveit::planning_interface::MoveGroupInterface>(camera_planning_group);

  moveit_control::MoveItClient moveit_control(
      nh_,
      arm_control_interface,
      gripper_control_interface,
      camera_control_interface,
      arm_planning_group);

  ROS_INFO_STREAM("MOVING TO HOME");
  moveit_control.goPreset("home");
  ROS_INFO_STREAM("DONE MOVING TO HOME!!!!!");
  std::this_thread::sleep_for(std::chrono::seconds(1));

  ROS_INFO_STREAM("MOVING TO HOVER");
  moveit_control.goPreset("hover");
  ROS_INFO_STREAM("DONE MOVING TO HOVER!!!!!");
  std::this_thread::sleep_for(std::chrono::seconds(1));

  ROS_INFO_STREAM("MOVING TO APPROACH");
  moveit_control.goPreset("approach");
  ROS_INFO_STREAM("DONE MOVING TO APPROACH!!!!!");
  std::this_thread::sleep_for(std::chrono::seconds(1));

  ROS_INFO_STREAM("MOVING TO GRIPPER TO SODA CAN PRESET");
  moveit_control.goPresetGripper("soda_can");
  ROS_INFO_STREAM("DONE MOVING TO GRIPPER TO SODA CAN PRESET!!!!!");

  ROS_WARN_STREAM("ATTACHING CAN PEPSI");
  moveit_control.attachGazeboModel("Coke", "link");
  ROS_WARN_STREAM("FINISHED ATTACHING CAN PEPSI");

  ROS_INFO_STREAM("MOVING TO HOVER AGAIN");
  moveit_control.goPreset("hover");
  ROS_INFO_STREAM("DONE MOVING TO HOVER AGAIN!!!!!");
  std::this_thread::sleep_for(std::chrono::seconds(1));

  auto exec1 = std::thread([&](){
    ROS_WARN_STREAM("OPENING GRIPPER");
    moveit_control.goPresetGripper("open");
    ROS_WARN_STREAM("FINISHED OPENING GRIPPER");
  });

  auto exec2 = std::thread([&](){
    ROS_WARN_STREAM("DETACHING CAN PEPSI");
    moveit_control.detachGazeboModel("Coke", "link");
    ROS_WARN_STREAM("FINISHED DETACHING CAN PEPSI");
  });

  if (exec1.joinable()) exec1.join();
  if (exec2.joinable()) exec2.join();

  moveit_control.printUsefulInfo();

  return 0;
}
