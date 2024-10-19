#include "task_handler/task_handler.hpp"
#include "ros/init.h"
#include "std_srvs/SetBoolRequest.h"
#include "std_srvs/SetBoolResponse.h"
#include "visualization_msgs/Marker.h"

TaskHandler::TaskHandler(ros::NodeHandle& nh, ros::NodeHandle& pnh)
  : nh_{nh}
{
  goal_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1, [this](const geometry_msgs::PoseStamped::ConstPtr& input){ this->goalCallback(input); });
  result_sub_ = nh_.subscribe<move_base_msgs::MoveBaseActionResult>("/move_base/result", 1, [this](const move_base_msgs::MoveBaseActionResult::ConstPtr& msg){ this->mbResultCallback(msg); });
  object_poses_sub_ = nh_.subscribe<task_handler::Objects>("/object_poses", 1, [this](const task_handler::Objects::ConstPtr& msg){ this->objectPosesCallback(msg); });

  state_pub_ = nh_.advertise<std_msgs::String>("/homebot/state", 1);
  task_pub_ = nh_.advertise<std_msgs::String>("/homebot/task", 1);
  goal_pub_ = nh_.advertise<visualization_msgs::Marker>("/homebot/goal", 1);

  ac_ = std::make_unique<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>>("move_base", true);
  while (!ac_->waitForServer(ros::Duration(5.0)))
  {
    ROS_INFO_STREAM("Waiting for the move_base action server to come up");
  }

  reset_srv_client_ = nh_.serviceClient<std_srvs::SetBool>("/homebot/reset");
  goal_srv_server_ = nh.advertiseService("/homebot/goal_task", &TaskHandler::uiButtonCallback, this);

  object_loc_map_["cup"] = "sink";
  object_loc_map_["book"] = "shelf";

  geometry_msgs::Pose pose;
  pose.position.x = 7.78;
  pose.position.y = -4.22;
  pose.orientation.z = -0.7;
  pose.orientation.w = 0.71;
  target_loc_map_["sink"] = pose;

  pose.position.x = 4.3;
  pose.position.y = -4.68;
  pose.orientation.z = -0.7;
  pose.orientation.w = 0.71;
  target_loc_map_["shelf"] = pose;

  curr_state_ = State::IDLE;
  state_str_.data = "Idle";
  state_pub_.publish(state_str_);
  task_str_.data = "Waiting for goal...";
  task_pub_.publish(task_str_);

  std_srvs::SetBool srv;
  srv.request.data = true;
  ros::Rate rate(1);
  while (ros::ok() && !reset_srv_client_.call(srv))
  {
    ROS_INFO_STREAM("Waiting to reset...");
    rate.sleep();
  }
}

TaskHandler::~TaskHandler()
{
}

void TaskHandler::goalCallback(const geometry_msgs::PoseStamped::ConstPtr& input)
{
  ROS_INFO_STREAM("New rviz goal received!");

  goal_ = *input;

  visualization_msgs::Marker marker;
  marker.header.frame_id = goal_.header.frame_id;
  marker.header.stamp = goal_.header.stamp;
  marker.id = 0;
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose = goal_.pose;
  marker.scale.x = 1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  goal_pub_.publish(marker);
}

bool TaskHandler::uiButtonCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
  // send mb goal
  if (req.data)
  {
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose = goal_;
    goal.target_pose.header.stamp = ros::Time::now();
    ac_->sendGoal(goal);
    res.success = true;
    res.message = "Goal sent";

    curr_state_ = State::GO_TO_GOAL;
    state_str_.data = "Go to goal";
    state_pub_.publish(state_str_);
    task_str_.data = "Executing plan...";
    task_pub_.publish(task_str_);
  }
  else // abort task
  {
    ac_->cancelAllGoals();
    res.success = true;
    res.message = "Goal aborted";

    curr_state_ = State::IDLE;
    state_str_.data = "Idle";
    state_pub_.publish(state_str_);
    task_str_.data = "Waiting for goal...";
    task_pub_.publish(task_str_);
  }

  return true;
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
        curr_state_ = State::READY_FOR_TASK;
        state_str_.data = "Ready for task";
        state_pub_.publish(state_str_);
        task_str_.data = "Detecting objects...";
        task_pub_.publish(task_str_);
        break;
      case State::GO_TO_OBJECT:
        curr_state_ = State::PICK_OBJECT;
        state_str_.data = "Pick object";
        state_pub_.publish(state_str_);
        pickObject();
        break;
      case State::GO_TO_TARGET_LOC:
        curr_state_ = State::PLACE_OBJECT;
        state_str_.data = "Place object";
        state_pub_.publish(state_str_);
        placeObject();
        break;
    }
  }
  else
  {
    ROS_WARN_STREAM("Failed to reach the goal. Status: " << msg->status.status);
    curr_state_ = State::IDLE;
    state_str_.data = "Idle";
    state_pub_.publish(state_str_);
    task_str_.data = "Waiting for goal...";
    task_pub_.publish(task_str_);
  }
}

void TaskHandler::objectPosesCallback(const task_handler::Objects::ConstPtr& msg)
{
  if (curr_state_ != State::READY_FOR_TASK)
    return;

  ROS_INFO_STREAM("Object poses received!");

  std::queue<task_handler::Object> empty;
  std::swap( objects_, empty );

  for (const auto& obj : msg->objects)
  {
    ROS_INFO_STREAM("Adding " << obj.name << " to list");
    objects_.push(obj);
  }

  curr_state_ = State::GO_TO_OBJECT;
  state_str_.data = "Go to object";
  state_pub_.publish(state_str_);
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

  task_str_.data = "Going to " + objects_.front().name + " position...";
  task_pub_.publish(task_str_);
}

void TaskHandler::pickObject()
{
  ROS_INFO_STREAM("Picking up object");

  // TODO: send moveit pick task

  // once done picking
  curr_state_ = State::GO_TO_TARGET_LOC;
  state_str_.data = "Go to Target";
  state_pub_.publish(state_str_);

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

  task_str_.data = "Going to " + object_loc_map_[objects_.front().name] + "...";
  task_pub_.publish(task_str_);
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
    state_str_.data = "Go to object";
    state_pub_.publish(state_str_);
    sendObjectGoal();
  }
  else
  {
    curr_state_ = State::IDLE;
    state_str_.data = "Idle";
    state_pub_.publish(state_str_);
    task_str_.data = "Waiting for goal...";
    task_pub_.publish(task_str_);
    ROS_INFO_STREAM("All tasks done!");

    std_srvs::SetBool srv;
    srv.request.data = true;
    ros::Rate rate(1);
    while (ros::ok() && !reset_srv_client_.call(srv))
    {
      ROS_INFO_STREAM("Waiting to reset...");
      rate.sleep();
    }
  }
}
