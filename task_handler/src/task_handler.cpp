#include "task_handler/task_handler.hpp"
#include "angles/angles.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "moveit_msgs/CollisionObject.h"
#include "tf2/LinearMath/Quaternion.h"

TaskHandler::TaskHandler(ros::NodeHandle& nh, ros::NodeHandle& pnh)
  : nh_{nh},
    tf2_listener{tf2_buffer},
    initialized_{false}
{
  static const std::string arm_planning_group = "stretch_arm";
  static const std::string gripper_planning_group = "stretch_gripper";
  static const std::string camera_planning_group = "stretch_head";

  auto arm_control_interface =
    std::make_shared<moveit::planning_interface::MoveGroupInterface>(arm_planning_group);
  auto gripper_control_interface =
    std::make_shared<moveit::planning_interface::MoveGroupInterface>(gripper_planning_group);
  auto camera_control_interface =
    std::make_shared<moveit::planning_interface::MoveGroupInterface>(camera_planning_group);

  moveit_control_ = std::make_unique<moveit_control::MoveItClient>
  (
    nh_,
    arm_control_interface,
    gripper_control_interface,
    camera_control_interface,
    arm_planning_group
  );

  result_sub_ = nh_.subscribe<move_base_msgs::MoveBaseActionResult>("/move_base/result", 1, [this](const move_base_msgs::MoveBaseActionResult::ConstPtr& msg){ this->mbResultCallback(msg); });
  object_poses_sub_ = nh_.subscribe<task_handler::Objects>("/object_poses", 1, [this](const task_handler::Objects::ConstPtr& msg){ this->objectPosesCallback(msg); });

  cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/stretch_diff_drive_vcontroller/cmd_vel", 1);
  state_pub_ = nh_.advertise<std_msgs::String>("/homebot/state", 1);
  task_pub_ = nh_.advertise<std_msgs::String>("/homebot/task", 1);
  goal_pub_ = nh_.advertise<visualization_msgs::Marker>("/homebot/goal", 1);

  reset_srv_client_ = nh_.serviceClient<std_srvs::SetBool>("/homebot/reset");
  goal_srv_server_ = nh.advertiseService("/homebot/goal_task", &TaskHandler::uiButtonCallback, this);

  ac_ = std::make_unique<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>>("move_base", true);
  while (!ac_->waitForServer(ros::Duration(5.0)))
  {
    ROS_INFO_STREAM("Waiting for the move_base action server to come up");
  }

  initLookups();

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

void TaskHandler::init()
{
  moveit_control_->goPresetGripper("closed");
  moveit_control_->goPresetGripper("open");
  moveit_control_->goPresetCamera("homebot");
}

bool TaskHandler::uiButtonCallback(task_handler::GoalTask::Request &req, task_handler::GoalTask::Response &res)
{
  if (!initialized_)
  {
    init();
    initialized_ = true;
  }

  // send mb goal
  if (req.active)
  {
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose = zone_goal_map_[req.zone];
    goal.target_pose.header.stamp = ros::Time::now();
    ac_->sendGoal(goal);
    res.success = true;
    res.message = "Goal sent";

    curr_state_ = State::GO_TO_GOAL;
    state_str_.data = "Go to Cleaning Zone";
    state_pub_.publish(state_str_);
    task_str_.data = "Executing plan...";
    task_pub_.publish(task_str_);

    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.id = 0;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose = goal.target_pose.pose;
    marker.scale.x = 1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    goal_pub_.publish(marker);

    std::queue<task_handler::Object> empty;
    std::swap( objects_, empty );
    std::queue<geometry_msgs::PoseStamped> empty_pose;
    std::swap( objects_approach_goals_, empty_pose );
    task_handler::Object obj;
    if (req.zone == "Coffee Table")
    {
      auto goal_pose = zone_goal_map_[req.zone];

      auto pose = moveit_control_->getGazeboModelPose("Coke");
      if (pose.has_value())
      {
        obj.id = "soda_can";
        obj.name = "Coke";
        obj.pose = pose.value();
        obj.pose.position.z += 0.07;
        objects_.push(obj);
        std::cout << "obj pose: " << obj.pose << std::endl;
        auto res = getApproachPose(goal_pose, obj.pose);
        std::cout << "app pose: " << res << std::endl;
        objects_approach_goals_.push(res);
      }

      pose = moveit_control_->getGazeboModelPose("Mug");
      if (pose.has_value())
      {
        obj.id = "cup";
        obj.name = "Mug";
        obj.pose = pose.value();
        obj.pose.position.z += 0.08;
        objects_.push(obj);
        objects_approach_goals_.push(getApproachPose(goal_pose, obj.pose));
      }
    }
    else if (req.zone == "Dining Table")
    {
      auto goal_pose = zone_goal_map_[req.zone];

      auto pose = moveit_control_->getGazeboModelPose("Book2");
      if (pose.has_value())
      {
        obj.id = "book";
        obj.name = "Book2";
        obj.pose = pose.value();
        obj.pose.position.z += 0.13;
        objects_.push(obj);
        objects_approach_goals_.push(getApproachPose(goal_pose, obj.pose));
      }

      pose = moveit_control_->getGazeboModelPose("Book");
      if (pose.has_value())
      {
        obj.id = "book";
        obj.name = "Book";
        obj.pose = pose.value();
        obj.pose.position.z += 0.13;
        objects_.push(obj);
        objects_approach_goals_.push(getApproachPose(goal_pose, obj.pose));
      }
    }
  }
  else // abort task
  {
    ac_->cancelAllGoals();
    moveit_control_->cancel();
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

        curr_state_ = State::GO_TO_OBJECT;
        state_str_.data = "Go to object";
        state_pub_.publish(state_str_);
        sendObjectGoal();
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

void TaskHandler::initLookups()
{
  /**
   * Object List
   ===========
   - Book
   - Book2
   - Coke
   - Mug 
   */

  // initialize the target locations for each object of interest
  object_location_map_.clear();
  object_target_map_.clear();
  robot_target_map_.clear();

  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = "map";

  // sink position
  pose.pose.position.x = 8.03;
  pose.pose.position.y = -5.13;
  pose.pose.position.z = 0.8;
  pose.pose.orientation.w = 1.0;
  object_location_map_["Mug"] = "sink";
  object_target_map_["Mug"] = pose;
  pose.pose.position.y = -4.43;
  pose.pose.position.z = 0.0;
  robot_target_map_["sink"] = pose;

  // book shelf position 1
  pose.pose.position.x = 4.407;
  pose.pose.position.y = -5.182;
  pose.pose.position.z = 0.85;
  pose.pose.orientation.w = 1.0;
  object_location_map_["Book2"] = "book shelf";
  object_target_map_["Book2"] = pose;
  pose.pose.position.y = -4.64;
  pose.pose.position.z = 0.0;
  robot_target_map_["book shelf"] = pose;

  // book shelf position 2
  pose.pose.position.x = 4.142;
  pose.pose.position.y = -5.182;
  pose.pose.position.z = 0.85;
  pose.pose.orientation.w = 1.0;
  object_location_map_["Book"] = "bookshelf";
  object_target_map_["Book"] = pose;
  pose.pose.position.y = -4.64;
  pose.pose.position.z = 0.0;
  robot_target_map_["bookshelf"] = pose;

  // bin position
  pose.pose.position.x = 2.36;
  pose.pose.position.y = -0.796;
  pose.pose.position.z = 0.6;
  pose.pose.orientation.z = -0.7;
  pose.pose.orientation.w = 0.71;
  object_location_map_["Coke"] = "bin";
  object_target_map_["Coke"] = pose;
  pose.pose.position.x = 2.8;
  pose.pose.position.z = 0.0;
  robot_target_map_["bin"] = pose;

  // initialize goal positions for each cleaning zone
  zone_goal_map_.clear();
  tf2::Quaternion quat;
  geometry_msgs::PoseStamped goal;
  goal.header.frame_id = "map";

  // coffee table
  goal.pose.position.x = 2.5;
  goal.pose.position.y = -1.628;
  goal.pose.position.z = 0.0;
  quat.setRPY(0.0, 0.0, -1.57);
  quat.normalize();
  goal.pose.orientation = tf2::toMsg(quat);
  zone_goal_map_["Coffee Table"] = goal;

  // dining table
  goal.pose.position.x = 5.34;
  goal.pose.position.y = 0.7;
  goal.pose.position.z = 0.0;
  quat.setRPY(0.0, 0.0, 1.57);
  quat.normalize();
  goal.pose.orientation = tf2::toMsg(quat);
  zone_goal_map_["Dining Table"] = goal;
}

void TaskHandler::sendObjectGoal()
{
  ROS_INFO_STREAM("Sending object goal");

  auto target_pose = objects_approach_goals_.front();
  target_pose.header.stamp = ros::Time::now();

  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose = target_pose;

  ac_->sendGoal(goal);

  task_str_.data = "Going to " + objects_.front().name + " position...";
  task_pub_.publish(task_str_);
}

void TaskHandler::pickObject()
{
  ROS_INFO_STREAM("Picking up object");

  auto object = objects_.front();
  geometry_msgs::PoseStamped object_stamped;
  object_stamped.header.frame_id = "map";
  object_stamped.header.stamp = ros::Time::now();
  object_stamped.pose = object.pose;

  task_str_.data = "Going to high arm position...";
  task_pub_.publish(task_str_);
  moveit_control_->goPreset("home_high");

  task_str_.data = "Hovering arm above object...";
  task_pub_.publish(task_str_);
  moveit_control_->hoverArm(object_stamped);

  task_str_.data = "Arm approaching object...";
  task_pub_.publish(task_str_);
  moveit_control_->approachArm(object_stamped);

  task_str_.data = "Closing gripper...";
  task_pub_.publish(task_str_);
  moveit_control_->goPresetGripper(object.id);

  task_str_.data = "Picking up object...";
  task_pub_.publish(task_str_);
  moveit_control_->attachGazeboModel(object.name, "link");

  task_str_.data = "Hovering arm...";
  task_pub_.publish(task_str_);
  moveit_control_->hoverArm(object_stamped);

  task_str_.data = "Going to high arm position...";
  task_pub_.publish(task_str_);
  moveit_control_->goPreset("home_high");

  // once done picking
  curr_state_ = State::GO_TO_TARGET_LOC;
  state_str_.data = "Go to Object Destination";
  state_pub_.publish(state_str_);

  auto target_pose = robot_target_map_[object_location_map_[object.name]];
  target_pose.header.stamp = ros::Time::now();

  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose = target_pose;

  ROS_INFO_STREAM("Sending destination: " << object_location_map_[object.name]);

  ac_->sendGoal(goal);

  task_str_.data = "Going to " + object_location_map_[objects_.front().name] + "...";
  task_pub_.publish(task_str_);
}

void TaskHandler::placeObject()
{
  ROS_INFO_STREAM("Placing object");

  auto object = objects_.front();

  auto target_pose = object_target_map_[object.name];
  target_pose.header.frame_id = "map";
  target_pose.header.stamp = ros::Time::now();

  task_str_.data = "Hovering arm...";
  task_pub_.publish(task_str_);
  moveit_control_->hoverArm(target_pose);

  task_str_.data = "Lowering arm...";
  task_pub_.publish(task_str_);
  moveit_control_->approachArm(target_pose);

  task_str_.data = "Placing down object...";
  task_pub_.publish(task_str_);
  auto exec1 = std::thread([&](){
    moveit_control_->goPresetGripper("open");
  });

  auto exec2 = std::thread([&](){
    moveit_control_->detachGazeboModel(object.name, "link");
  });

  if (exec1.joinable()) exec1.join();
  if (exec2.joinable()) exec2.join();

  task_str_.data = "Hovering arm...";
  task_pub_.publish(task_str_);
  moveit_control_->hoverArm(target_pose);

  task_str_.data = "Going to high arm position...";
  task_pub_.publish(task_str_);
  moveit_control_->goPreset("home_high");

  task_str_.data = "Going to low arm position...";
  task_pub_.publish(task_str_);
  moveit_control_->goPreset("home_low");

  // once done placing
  objects_.pop();
  objects_approach_goals_.pop();
  if (!objects_.empty())
  {
    // send the next object
    curr_state_ = State::GO_TO_OBJECT;
    state_str_.data = "Go to next object";
    state_pub_.publish(state_str_);
    sendObjectGoal();
  }
  else
  {
    curr_state_ = State::IDLE;
    state_str_.data = "Idle";
    state_pub_.publish(state_str_);
    task_str_.data = "All tasks done!";
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

geometry_msgs::PoseStamped TaskHandler::getApproachPose(const geometry_msgs::PoseStamped& goal_pose, const geometry_msgs::Pose& obj_pose)
{
  geometry_msgs::PoseStamped approach_pose;
  approach_pose.header.frame_id = goal_pose.header.frame_id;
  approach_pose.pose.orientation = goal_pose.pose.orientation;

  // find rotation and translation matrices to transform from map frame to goal pose frame
  auto yaw = tf2::getYaw(goal_pose.pose.orientation);
  Eigen::Rotation2D<double> r(-yaw);
  Eigen::Rotation2D<double> ir(yaw);
  Eigen::Translation<double, 2> t(-goal_pose.pose.position.x, -goal_pose.pose.position.y);
  Eigen::Translation<double, 2> it(goal_pose.pose.position.x, goal_pose.pose.position.y);

  // find object position in goal pose frame
  Eigen::Vector2d obj_position;
  obj_position << obj_pose.position.x, obj_pose.position.y;
  obj_position = t * obj_position;
  obj_position = r * obj_position;

  obj_position(0) += 0.021;
  obj_position(1) = 0.0;

  obj_position = ir * obj_position;
  obj_position = it * obj_position;

  approach_pose.pose.position.x = obj_position(0);
  approach_pose.pose.position.y = obj_position(1);
  approach_pose.pose.position.z = 0.0;

  return approach_pose;
}

