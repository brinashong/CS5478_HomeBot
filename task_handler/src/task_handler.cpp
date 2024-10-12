#include "task_handler/task_handler.hpp"

TaskHandler::TaskHandler(ros::NodeHandle& nh, ros::NodeHandle& pnh)
  : nh_{nh}
{
  goal_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1, [this](const geometry_msgs::PoseStamped::ConstPtr& input){ this->goalCallback(input); });
  result_sub_ = nh_.subscribe<move_base_msgs::MoveBaseActionResult>("/move_base/result", 1, [this](const move_base_msgs::MoveBaseActionResult::ConstPtr& msg){ this->mbResultCallback(msg); });
  object_poses_sub_ = nh_.subscribe<task_handler::Objects>("/object_poses", 1, [this](const task_handler::Objects::ConstPtr& msg){ this->objectPosesCallback(msg); });

  ac_ = std::make_unique<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>>("move_base", true);
  while (!ac_->waitForServer(ros::Duration(5.0)))
  {
    ROS_INFO_STREAM("Waiting for the move_base action server to come up");
  }

  object_loc_map_["cup"] = "sink";
  object_loc_map_["book"] = "shelf";

  geometry_msgs::Pose pose;
  pose.position.x = 8.0;
  pose.position.y = -4.58;
  pose.orientation.z = -0.7;
  pose.orientation.w = 0.71;
  target_loc_map_["sink"] = pose;

  pose.position.x = 4.3;
  pose.position.y = -4.68;
  pose.orientation.z = -0.7;
  pose.orientation.w = 0.71;
  target_loc_map_["shelf"] = pose;

  curr_state_ = State::IDLE;
}

TaskHandler::~TaskHandler()
{
}

void TaskHandler::goalCallback(const geometry_msgs::PoseStamped::ConstPtr& input)
{
  ROS_INFO_STREAM("New rviz goal received!");

  if (curr_state_ == State::GO_TO_TARGET_LOC)
  {
    ac_->cancelAllGoals();
  }

  curr_state_ = State::GO_TO_GOAL;
}

void TaskHandler::mbResultCallback(const move_base_msgs::MoveBaseActionResult::ConstPtr& msg)
{
  ROS_INFO_STREAM("Move Base result received");

  if (msg->status.status == 3)
  {
    ROS_INFO_STREAM("Goal reached successfully!");
    switch (curr_state_)
    {
      case State::GO_TO_GOAL:
        curr_state_ = State::READY;
        break;
      case State::GO_TO_OBJECT:
        curr_state_ = State::PICK_OBJECT;
        pickObject();
        break;
      case State::GO_TO_TARGET_LOC:
        curr_state_ = State::PLACE_OBJECT;
        placeObject();
        break;
    }
  }
  else
  {
    ROS_WARN_STREAM("Failed to reach the goal. Status: " << msg->status.status);
    curr_state_ = IDLE;
  }
}

void TaskHandler::objectPosesCallback(const task_handler::Objects::ConstPtr& msg)
{
  if (curr_state_ != State::READY)
    return;

  ROS_INFO_STREAM("Object poses received!");

  std::queue<task_handler::Object> empty;
  std::swap( objects_, empty );

  for (const auto& obj : msg->objects)
  {
    objects_.push(obj);
  }

  curr_state_ = State::GO_TO_OBJECT;
  sendObjectGoal();
}

void TaskHandler::sendObjectGoal()
{
  // TODO: find suitable approach coordinates
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = objects_.front().pose.position.x;
  goal.target_pose.pose.position.y = objects_.front().pose.position.y;
  goal.target_pose.pose.orientation.w = 1.0;

  ROS_INFO_STREAM("Sending object goal");

  ac_->sendGoal(goal);
}

void TaskHandler::pickObject()
{
  ROS_INFO_STREAM("Picking up object");

  // TODO: send moveit pick task

  // once done picking
  curr_state_ = State::GO_TO_TARGET_LOC;

  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = target_loc_map_[object_loc_map_[objects_.front().name]].position.x;
  goal.target_pose.pose.position.y = target_loc_map_[object_loc_map_[objects_.front().name]].position.y;
  goal.target_pose.pose.orientation.x = target_loc_map_[object_loc_map_[objects_.front().name]].orientation.x;
  goal.target_pose.pose.orientation.y = target_loc_map_[object_loc_map_[objects_.front().name]].orientation.y;
  goal.target_pose.pose.orientation.z = target_loc_map_[object_loc_map_[objects_.front().name]].orientation.z;
  goal.target_pose.pose.orientation.w = target_loc_map_[object_loc_map_[objects_.front().name]].orientation.w;

  ROS_INFO_STREAM("Sending target destination");

  ac_->sendGoal(goal);
}

void TaskHandler::placeObject()
{
  ROS_INFO_STREAM("Placing object");

  // TODO: send moveit place task

  // once done placing
  objects_.pop();
  if (!objects_.empty())
  {
    // send the next object
    curr_state_ = State::GO_TO_OBJECT;
    sendObjectGoal();
  }
  else
  {
    curr_state_ = State::IDLE;
  }
}
