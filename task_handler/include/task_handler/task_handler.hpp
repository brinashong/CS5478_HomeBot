#ifndef TASK_HANDLER_HPP
#define TASK_HANDLER_HPP

#include <memory>
#include <queue>
#include <algorithm>
#include <unordered_map>

#include <ros/ros.h>
#include <ros/service_server.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseActionResult.h>
#include <std_srvs/SetBool.h>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>

#include "task_handler/Object.h"
#include "task_handler/Objects.h"

class TaskHandler
{
public:
  enum State
  {
    IDLE,
    GO_TO_GOAL,
    READY_FOR_TASK,
    GO_TO_OBJECT,
    PICK_OBJECT,
    GO_TO_TARGET_LOC,
    PLACE_OBJECT
  };

  TaskHandler(ros::NodeHandle& nh, ros::NodeHandle& pnh);
  ~TaskHandler();
  void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& input);
  void mbResultCallback(const move_base_msgs::MoveBaseActionResult::ConstPtr& msg);
  void objectPosesCallback(const task_handler::Objects::ConstPtr& msg);
  bool uiButtonCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);

private:
  void sendObjectGoal();
  void pickObject();
  void placeObject();
  void moveArm(const double height, const double depth);
  void moveGripper(const std::string& target);
  Eigen::Isometry3d poseMsgToEigen(const geometry_msgs::Pose& msg);

  ros::NodeHandle nh_;
  ros::Subscriber goal_sub_;
  ros::Subscriber result_sub_;
  ros::Subscriber object_poses_sub_;
  ros::Publisher state_pub_;
  ros::Publisher task_pub_;
  ros::Publisher goal_pub_;
  ros::ServiceClient reset_srv_client_;
  ros::ServiceServer goal_srv_server_;
  std::unique_ptr<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>> ac_;
  moveit::planning_interface::MoveGroupInterface arm_group_, gripper_group_;

  State curr_state_;
  std_msgs::String state_str_;
  std_msgs::String task_str_;
  geometry_msgs::PoseStamped goal_;
  std::queue<task_handler::Object> objects_;
  std::unordered_map<std::string, std::string> object_loc_map_;
  std::unordered_map<std::string, geometry_msgs::Pose> target_loc_map_;
};

#endif
