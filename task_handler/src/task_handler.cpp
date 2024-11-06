#include "task_handler/task_handler.hpp"

TaskHandler::TaskHandler(ros::NodeHandle& nh, ros::NodeHandle& pnh)
  : nh_{nh},
    arm_group_{"stretch_arm"},
    gripper_group_{"stretch_gripper"},
    tf2_listener{tf2_buffer}
{
  arm_group_.setGoalPositionTolerance(0.1);
  arm_group_.setGoalOrientationTolerance(0.1);
  arm_group_.setGoalJointTolerance(0.001);
  // arm_group_.setPlannerId("RRTConnectkConfigDefault");
  arm_group_.setPlanningTime(10.0);

  gripper_group_.setGoalPositionTolerance(0.001);
  gripper_group_.setGoalOrientationTolerance(0.01);
  gripper_group_.setGoalJointTolerance(0.001);
  gripper_group_.setPlannerId("TRRT");
  gripper_group_.setPlanningTime(10.0);

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
  object_loc_map_["can"] = "bin";

  geometry_msgs::Pose pose;
  pose.position.x = 8.2;
  pose.position.y = -4.35;
  pose.position.z = 1.5;
  pose.orientation.z = -0.7;
  pose.orientation.w = 0.71;
  target_loc_map_["sink"] = pose;

  pose.position.x = 4.32;
  pose.position.y = -4.55;
  pose.position.z = 1.2;
  pose.orientation.z = -0.7;
  pose.orientation.w = 0.71;
  target_loc_map_["shelf"] = pose;

  pose.position.x = 2.82;
  pose.position.y = -0.78;
  pose.position.z = 0.5;
  pose.orientation.x = 0.0;
  pose.orientation.y = 1.0;
  pose.orientation.z = 0.0;
  pose.orientation.w = 0.0;
  target_loc_map_["bin"] = pose;

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

  auto object = objects_.front();

  // move TCP to above object
  auto pose_in_base_link = poseInBaseLink(object.pose);
  moveArm(pose_in_base_link.pose.position.z + 0.2, pose_in_base_link.pose.position.y);

  // open the gripper
  moveGripper("open");

  // move the TCP close to the object
  moveArm(pose_in_base_link.pose.position.z, pose_in_base_link.pose.position.y);

  // close the gripper
  moveGripper("closed");

  // TODO: lift it up a bit
  moveArm(pose_in_base_link.pose.position.z + 0.2, pose_in_base_link.pose.position.y);

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

  auto target_pose = target_loc_map_[objects_.front().name];

  // Move the TCP above
  auto pose_in_base_link = poseInBaseLink(target_pose);
  moveArm(pose_in_base_link.pose.position.z + 0.2, pose_in_base_link.pose.position.y);

  // Lower the TCP
  moveArm(pose_in_base_link.pose.position.z, pose_in_base_link.pose.position.y);

  // Open the gripper
  moveGripper("open");

  // lift the arm a bit
  moveArm(pose_in_base_link.pose.position.z + 0.2, pose_in_base_link.pose.position.y);

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

void TaskHandler::moveArm(const double height, const double depth)
{
  // auto curr_pose = arm_group_.getCurrentPose(arm_group_.getEndEffectorLink());

  geometry_msgs::Pose target_pose; // = curr_pose.pose;
  target_pose.position.x = 0.0;
  target_pose.position.y = depth;
  target_pose.position.z = height;
  target_pose.orientation.x = 1.0;
  target_pose.orientation.y = 0.0;
  target_pose.orientation.z = 0.0;
  target_pose.orientation.w = 0.0;

  arm_group_.setStartStateToCurrentState();  
  arm_group_.setApproximateJointValueTarget(poseMsgToEigen(target_pose), arm_group_.getEndEffectorLink());

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = (arm_group_.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

  if (success)
  {
    ROS_INFO("Executing arm movement...");
    arm_group_.execute(plan);
  }
  else
  {
    ROS_WARN("Arm movement planning failed.");
  }
}

void TaskHandler::moveGripper(const std::string& target)
{
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  gripper_group_.setJointValueTarget(gripper_group_.getNamedTargetValues(target));

  bool success = (gripper_group_.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

  if (success)
  {
    ROS_INFO("Executing gripper movement...");
    gripper_group_.execute(plan);
  }
  else
  {
    ROS_WARN("Gripper movement planning failed.");
  }
}

Eigen::Isometry3d TaskHandler::poseMsgToEigen(const geometry_msgs::Pose& msg)
{
  Eigen::Isometry3d approx_target;
  Eigen::Translation3d translation(msg.position.x, msg.position.y, msg.position.z);
  Eigen::Quaterniond quaternion(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z);
  if ((quaternion.x() == 0) && (quaternion.y() == 0) && (quaternion.z() == 0) && (quaternion.w() == 0))
  {
    ROS_WARN("Empty quaternion found in pose message. Setting to neutral orientation.");
    quaternion.setIdentity();
  }
  else
  {
    quaternion.normalize();
  }
  approx_target = translation * quaternion;
  return approx_target;
}

geometry_msgs::PoseStamped TaskHandler::poseInBaseLink(const geometry_msgs::Pose& pose)
{
  // transform from map to baselink frame
  geometry_msgs::PoseStamped pose_in_map;
  pose_in_map.header.frame_id = "map";
  pose_in_map.header.stamp = ros::Time(0);
  pose_in_map.pose = pose;

  geometry_msgs::PoseStamped pose_in_base_link;

  try
  {
    tf2_buffer.canTransform("base_link", "map", ros::Time(0), ros::Duration(3.0));

    pose_in_base_link = tf2_buffer.transform(pose_in_map, "base_link");

    ROS_INFO_STREAM("Transformed Pose: " << pose_in_base_link);
  }
  catch (tf2::TransformException &ex)
  {
    ROS_WARN("Could not transform pose: %s", ex.what());
  }

  return pose_in_base_link;
}
