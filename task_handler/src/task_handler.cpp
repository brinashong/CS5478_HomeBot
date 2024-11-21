#include "task_handler/task_handler.hpp"

TaskHandler::TaskHandler(ros::NodeHandle& nh, ros::NodeHandle& pnh)
  : nh_{nh},
    arm_group_{"stretch_arm"},
    gripper_group_{"stretch_gripper"},
    tf2_listener{tf2_buffer}
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

bool TaskHandler::uiButtonCallback(task_handler::GoalTask::Request &req, task_handler::GoalTask::Response &res)
{
  // send mb goal
  if (req.active)
  {
    std::cout << "received: " << req.goal.pose.position.x << " " << req.goal.pose.position.y << std::endl;

    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose = req.goal;
    goal.target_pose.header.stamp = ros::Time::now();
    ac_->sendGoal(goal);
    res.success = true;
    res.message = "Goal sent";

    curr_state_ = State::GO_TO_GOAL;
    state_str_.data = "Go to goal";
    state_pub_.publish(state_str_);
    task_str_.data = "Executing plan...";
    task_pub_.publish(task_str_);

    visualization_msgs::Marker marker;
    marker.header.frame_id = req.goal.header.frame_id;
    marker.header.stamp = goal.target_pose.header.stamp;
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

    // TEMP
    std::queue<task_handler::Object> empty;
    std::swap( objects_, empty );
    if (req.zone == "Coffee Table")
    {
      task_handler::Object obj;

      auto pose = moveit_control_->getGazeboModelPose("Coke1");
      if (pose.has_value())
      {
        obj.id = "can";
        obj.name = "Coke1";
        obj.pose = pose.value();
        objects_.push(obj);
      }

      pose = moveit_control_->getGazeboModelPose("Coke2");
      if (pose.has_value())
      {
        obj.id = "can";
        obj.name = "Coke2";
        obj.pose = pose.value();
        objects_.push(obj);
      }

      pose = moveit_control_->getGazeboModelPose("LivingRoomMug");
      if (pose.has_value())
      {
        obj.id = "cup";
        obj.name = "LivingRoomMug";
        obj.pose = pose.value();
        objects_.push(obj);
      }

    }
    else if (req.zone == "Dining Table")
    {
      task_handler::Object obj;

      auto pose = moveit_control_->getGazeboModelPose("Book");
      if (pose.has_value())
      {
        obj.id = "book";
        obj.name = "Book";
        obj.pose = pose.value();
        objects_.push(obj);
      }

      pose = moveit_control_->getGazeboModelPose("SonyBox");
      if (pose.has_value())
      {
        obj.id = "book";
        obj.name = "SonyBox";
        obj.pose = pose.value();
        objects_.push(obj);
      }
    }
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

        // TEMP
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
  // get object pose in baselink frame
  auto pose_in_base_link = poseInBaseLink(objects_.front().pose); 

  // // get baselink pose
  // geometry_msgs::TransformStamped transform;
  // geometry_msgs::PoseStamped baselink_pose;
  // try
  // {
  //   transform = tf2_buffer.lookupTransform("map", "base_link", ros::Time(0), ros::Duration(3.0));
  //
  //   baselink_pose.header = transform.header;
  //   baselink_pose.pose.position.x = transform.transform.translation.x;
  //   baselink_pose.pose.position.y = transform.transform.translation.y;
  //   baselink_pose.pose.position.z = transform.transform.translation.z;
  //   baselink_pose.pose.orientation.x = transform.transform.rotation.x;
  //   baselink_pose.pose.orientation.y = transform.transform.rotation.y;
  //   baselink_pose.pose.orientation.z = transform.transform.rotation.z;
  //   baselink_pose.pose.orientation.w = transform.transform.rotation.w;
  //
  //   ROS_INFO_STREAM("Baselink Pose: " << baselink_pose);
  // }
  // catch (tf2::TransformException &ex)
  // {
  //   ROS_WARN("Could not transform pose: %s", ex.what());
  //   return;
  // }

  auto delta = pose_in_base_link;
  delta.pose.position.y = 0.0;
  delta.pose.orientation.w = 1.0;

  geometry_msgs::PoseStamped target;
  try
  {
    tf2_buffer.canTransform("map", "base_link", ros::Time(0), ros::Duration(3.0));

    target = tf2_buffer.transform(delta, "map");

    ROS_INFO_STREAM("Transformed Pose: " << target);
  }
  catch (tf2::TransformException &ex)
  {
    ROS_WARN("Could not transform pose: %s", ex.what());
    return;
  }

  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose = target;

  ROS_INFO_STREAM("Sending object goal");

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

  moveit_control_->goPreset("home");
  object_stamped.pose = object.pose;
  moveit_control_->hoverArm(object_stamped);
  moveit_control_->approachArm(object_stamped);
  moveit_control_->goPresetGripper(object.id);
  moveit_control_->attachGazeboModel(object.name, "link");
  moveit_control_->hoverArm(object_stamped);
  moveit_control_->goPreset("home");

  // // move TCP to above object
  // auto pose_in_base_link = poseInBaseLink(object.pose); moveArm(pose_in_base_link.pose.position.z + 0.2, pose_in_base_link.pose.position.y);
  //
  // // open the gripper
  // moveGripper("open");
  //
  // // move the TCP close to the object
  // moveArm(pose_in_base_link.pose.position.z, pose_in_base_link.pose.position.y);
  //
  // // close the gripper
  // moveGripper("closed");
  //
  // // TODO: lift it up a bit
  // moveArm(pose_in_base_link.pose.position.z + 0.2, pose_in_base_link.pose.position.y);

  // once done picking
  curr_state_ = State::GO_TO_TARGET_LOC;
  state_str_.data = "Go to Target";
  state_pub_.publish(state_str_);

  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose = target_loc_map_[object_loc_map_[objects_.front().id]];

  ROS_INFO_STREAM("Sending target destination");

  ac_->sendGoal(goal);

  task_str_.data = "Going to " + object_loc_map_[objects_.front().id] + "...";
  task_pub_.publish(task_str_);
}

void TaskHandler::placeObject()
{
  ROS_INFO_STREAM("Placing object");

  auto object = objects_.front();
  geometry_msgs::PoseStamped target_pose;
  target_pose.pose = target_loc_map_[object.id];
  target_pose.header.frame_id = "map";
  target_pose.header.stamp = ros::Time::now();

  moveit_control_->hoverArm(target_pose);
  moveit_control_->approachArm(target_pose);

  auto exec1 = std::thread([&](){
    moveit_control_->goPresetGripper("open");
  });

  auto exec2 = std::thread([&](){
    moveit_control_->detachGazeboModel(object.name, "link");
  });

  if (exec1.joinable()) exec1.join();
  if (exec2.joinable()) exec2.join();

  moveit_control_->goPreset("home");

  // // Move the TCP above
  // auto pose_in_base_link = poseInBaseLink(target_pose);
  // moveArm(pose_in_base_link.pose.position.z + 0.2, pose_in_base_link.pose.position.y);
  //
  // // Lower the TCP
  // moveArm(pose_in_base_link.pose.position.z, pose_in_base_link.pose.position.y);
  //
  // // Open the gripper
  // moveGripper("open");
  //
  // // lift the arm a bit
  // moveArm(pose_in_base_link.pose.position.z + 0.2, pose_in_base_link.pose.position.y);

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
