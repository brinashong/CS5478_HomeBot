#ifndef TASK_HANDLER_HPP
#define TASK_HANDLER_HPP

#include <memory>
#include <queue>
#include <algorithm>
#include <unordered_map>

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseActionResult.h>

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

private:
  void sendObjectGoal();
  void pickObject();
  void placeObject();

  ros::NodeHandle nh_;
  ros::Subscriber goal_sub_;
  ros::Subscriber result_sub_;
  ros::Subscriber object_poses_sub_;
  std::unique_ptr<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>> ac_;

  State curr_state_;
  std::queue<task_handler::Object> objects_;
  std::unordered_map<std::string, std::string> object_loc_map_;
  std::unordered_map<std::string, geometry_msgs::Pose> target_loc_map_;
};

#endif
