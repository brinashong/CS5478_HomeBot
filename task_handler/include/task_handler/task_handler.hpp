#ifndef TASK_HANDLER_HPP
#define TASK_HANDLER_HPP

#include <memory>
#include <queue>
#include <algorithm>
#include <unordered_map>

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/TransformStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseActionResult.h>
#include <std_srvs/SetBool.h>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <angles/angles.h>

#include "task_handler/Object.h"
#include "task_handler/Objects.h"
#include "task_handler/GetObjects.h"
#include "task_handler/GoalTask.h"
#include "task_handler/moveit_client.hpp"

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
  bool uiButtonCallback(task_handler::GoalTask::Request &req, task_handler::GoalTask::Response &res);
  void objectPosesCallback(const task_handler::Objects::ConstPtr& msg);
  void mbResultCallback(const move_base_msgs::MoveBaseActionResult::ConstPtr& msg);
  void objectsCallback(const task_handler::Objects::ConstPtr& msg);
  void init();

private:
  void initLookups();
  void sendObjectGoal();
  void pickObject();
  void placeObject();
  geometry_msgs::PoseStamped getApproachPose(const geometry_msgs::PoseStamped& goal_pose, const geometry_msgs::Pose& obj_pose);

  ros::NodeHandle nh_;
  ros::Subscriber result_sub_;
  ros::Subscriber objects_sub_;
  ros::Publisher cmd_vel_pub_;
  ros::Publisher state_pub_;
  ros::Publisher task_pub_;
  ros::Publisher goal_pub_;
  ros::ServiceClient reset_srv_client_;
  ros::ServiceClient get_objects_srv_client_;
  ros::ServiceServer goal_srv_server_;

  tf2_ros::Buffer tf2_buffer;
  tf2_ros::TransformListener tf2_listener;

  std::unique_ptr<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>> ac_;
  std::unique_ptr<moveit_control::MoveItClient> moveit_control_;

  bool initialized_;

  State curr_state_;
  std::string curr_zone_;
  std_msgs::String state_str_;
  std_msgs::String task_str_;

  std::unordered_map<std::string, std::string> object_location_map_;
  std::unordered_map<std::string, geometry_msgs::PoseStamped> object_target_map_;
  std::unordered_map<std::string, geometry_msgs::PoseStamped> robot_target_map_;
  std::unordered_map<std::string, geometry_msgs::PoseStamped> zone_goal_map_;

  std::queue<task_handler::Object> objects_;
  std::queue<geometry_msgs::PoseStamped> objects_approach_goals_;
};

#endif
