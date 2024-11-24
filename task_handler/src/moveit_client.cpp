#include "task_handler/moveit_client.hpp"

namespace moveit_control
{
  MoveItClient::MoveItClient(const ros::NodeHandle& n,
      std::shared_ptr<moveit::planning_interface::MoveGroupInterface> control,
      std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper_control,
      std::shared_ptr<moveit::planning_interface::MoveGroupInterface> camera_control,
      const std::string& planning_group,
      std::vector<moveit_msgs::JointConstraint> default_joint_contraints)
    : n_{n}
    , control_{control}
    , gripper_control_{gripper_control}
    , camera_control_{camera_control}
    , default_joint_contraints_{default_joint_contraints}
    , initialize_{false}
    , position_tolerance_{0.01}
    , orientation_tolerance_{0.1}
    , joint_tolerance_{0.01}
    , max_acc_scale_factor_{1.0}
    , max_vel_scale_factor_{1.0}
    , num_planning_attempt_{10}
    , end_effector_link_{""}
    , reference_frame_{"base_link"}
    , planning_time_{10.0}
    , joint_model_group_(control_->getCurrentState()->getJointModelGroup(planning_group))
    , tf_buffer_{std::make_unique<tf2_ros::Buffer>()}
    , transform_listener_{std::make_shared<tf2_ros::TransformListener>(*tf_buffer_)}
  {
    // Arm
    control_->setGoalPositionTolerance(position_tolerance_);
    control_->setGoalOrientationTolerance(orientation_tolerance_);
    control_->setGoalJointTolerance(joint_tolerance_);

    control_->setMaxAccelerationScalingFactor(max_acc_scale_factor_);
    control_->setMaxVelocityScalingFactor(max_vel_scale_factor_);
    control_->setNumPlanningAttempts(num_planning_attempt_);

    const moveit::core::JointModelGroup* joint_model_group =
      control_->getCurrentState()->getJointModelGroup(planning_group);

    end_effector_link_ = control_->getEndEffectorLink();
    ROS_INFO_STREAM("End Effector Link: " << end_effector_link_);

    control_->setPoseReferenceFrame(end_effector_link_);

    control_->allowReplanning(true);
    control_->setPlanningTime(planning_time_);
    control_->setPlannerId("RRTConnect");

    // Gripper
    gripper_control_->setGoalPositionTolerance(position_tolerance_);
    gripper_control_->setGoalOrientationTolerance(orientation_tolerance_);
    gripper_control_->setGoalJointTolerance(joint_tolerance_);

    gripper_control_->setMaxAccelerationScalingFactor(max_acc_scale_factor_);
    gripper_control_->setMaxVelocityScalingFactor(max_vel_scale_factor_);
    gripper_control_->setNumPlanningAttempts(num_planning_attempt_);

    gripper_control_->allowReplanning(true);
    gripper_control_->setPlanningTime(planning_time_);
    gripper_control_->setPlannerId("RRTConnect");

    // Camera
    camera_control_->setGoalPositionTolerance(position_tolerance_);
    camera_control_->setGoalOrientationTolerance(orientation_tolerance_);
    camera_control_->setGoalJointTolerance(joint_tolerance_);

    camera_control_->setMaxAccelerationScalingFactor(max_acc_scale_factor_);
    camera_control_->setMaxVelocityScalingFactor(max_vel_scale_factor_);
    camera_control_->setNumPlanningAttempts(num_planning_attempt_);

    camera_control_->allowReplanning(true);
    camera_control_->setPlanningTime(planning_time_);
    camera_control_->setPlannerId("RRTConnect");

    gz_get_client_ = n_.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
    gz_set_client_ = n_.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");

    attach_gz_client_ = n_.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/attach");
    detach_gz_client_ = n_.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/detach");

    initialize_ = true;
  }

  bool MoveItClient::initCheck()
  {
    if (!initialize_)
    {
      ROS_WARN_STREAM(__func__ << ": Please init before calling methods!");
      return false;
    }
    return true;
  }

  bool MoveItClient::addCollision(const moveit_msgs::CollisionObject& _collision_object)
  {
    if (!initCheck()) return false;

    if (planning_scene_pub_.getNumSubscribers() == 0u)
    {
      ROS_WARN_STREAM(__func__ << ": Planning scene not available, please try again later.");
      return false;
    }

    std::vector<moveit_msgs::CollisionObject> collision_objects;
    auto collision_object = _collision_object;
    collision_object.operation = collision_object.ADD;
    collision_objects.push_back(collision_object);
    // planning_scene_interface_.addCollisionObjects(_collision_objects);
    planning_scene_interface_.applyCollisionObjects(collision_objects);

    std::cout << "Adding object " << collision_object.id
      << " to planning scence" << std::endl;

    // Debug
    std::cout << "Know Objects: ";
    for (const auto& obj_name : planning_scene_interface_.getKnownObjectNames())
    {
      std::cout << obj_name << ", ";
    }
    std::cout << std::endl;

    return true;
  }

  bool MoveItClient::addCollisions(const std::vector<moveit_msgs::CollisionObject>& _collision_objects)
  {
    if (!initCheck()) return false;

    if (planning_scene_pub_.getNumSubscribers() == 0u)
    {
      ROS_WARN_STREAM(__func__ << ": Planning scene not available, please try again later.");
      return false;
    }

    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects = _collision_objects;
    // planning_scene_interface_.addCollisionObjects(collision_objects);
    planning_scene_interface_.applyCollisionObjects(collision_objects);

    // Debug
    std::cout << "Know Objects: ";
    for (const auto& obj_name : planning_scene_interface_.getKnownObjectNames())
    {
      std::cout << obj_name << ", ";
    }
    std::cout << std::endl;

    return true;
  }

  bool MoveItClient::removeCollision(const std::string& target)
  {
    if (!initCheck()) return false;

    if (planning_scene_pub_.getNumSubscribers() == 0u)
    {
      ROS_WARN_STREAM(__func__ << ": Planning scene not available, please try again later.");
      return false;
    }

    // planning_scene_interface_.removeCollisionObjects({target});
    moveit_msgs::CollisionObject collision_object;
    collision_object.id = target;
    collision_object.operation = collision_object.REMOVE;
    planning_scene_interface_.applyCollisionObjects({collision_object});

    std::cout << "Removing collision object "
      << collision_object.id << " from planning scene." << std::endl;

    return true;
  }

  bool MoveItClient::clearCollisions()
  {
    if (!initCheck()) return false;

    if (planning_scene_pub_.getNumSubscribers() == 0u)
    {
      ROS_WARN_STREAM(__func__ << ": Planning scene not available, please try again later.");
      return false;
    }

    // Woah!
    planning_scene_interface_.clear();

    // // Ensures all objects are retrieved, typed or untyped.
    // auto ids = planning_scene_interface_.getKnownObjectNames(false);
    //
    // std::vector<moveit_msgs::CollisionObject> collision_objects;
    // for (const auto& id : ids)
    // {
    //   moveit_msgs::CollisionObject collision_object;
    //   collision_object.id = id;
    //   collision_object.operation = moveit_msgs::CollisionObject::REMOVE;
    //   collision_objects.push_back(collision_object);
    // }
    //
    // planning_scene_interface_.applyCollisionObjects(collision_objects);
    // std::cout << "All collision objects removed using applyCollisionObjects."
    //   << std::endl;

    ROS_INFO_STREAM(__func__ << ": All collision and attached objects have been removed"
        " from planning scene.");

    return true;
  }

  bool MoveItClient::attachObject(const std::string& _object_name,
        const moveit_msgs::AttachedCollisionObject& _attached_object,
        const std::string& _group_name
      )
  {
    if (!initCheck()) return false;

    if (planning_scene_pub_.getNumSubscribers() == 0u)
    {
      ROS_WARN_STREAM(__func__ << ": Planning scene not available, please try again later.");
      return false;
    }

    // Step 0: Prep attach object
    moveit_msgs::AttachedCollisionObject attach_object = _attached_object;
    attach_object.link_name = end_effector_link_;
    attach_object.object.header.frame_id = control_->getPlanningFrame();
    attach_object.object.operation = moveit_msgs::CollisionObject::ADD;

    // Step 1: Add collision object in planning scene
    addCollision(attach_object.object);

    // Step 2: Attach collision object to gripper
    // Sanity check whether step 1 was successful
    if (auto ids = planning_scene_interface_.getKnownObjectNames(false);
        std::find(ids.begin(), ids.end(), attach_object.object.id) != ids.end())
    {
      if (planning_scene_interface_.applyAttachedCollisionObject(attach_object))
      {
        ROS_INFO_STREAM(__func__ << ": Successfully attached object to end effector link.");
        return true;
      }
      else
      {
        ROS_WARN_STREAM(__func__ << ": Failed to attach object to end effector link.");
        return false;
      }
    }
    else
    {
      ROS_WARN_STREAM(__func__ << ": Failed to find collision object before attaching.");
      return false;
    }
  }

  bool MoveItClient::detachObject(const std::string& id)
  {
    if (!initCheck()) return false;

    if (planning_scene_pub_.getNumSubscribers() == 0u)
    {
      ROS_WARN_STREAM(__func__ << ": Planning scene not available, please try again later.");
      return false;
    }

    // Step 0: Prep detach object
    moveit_msgs::AttachedCollisionObject detach_object;
    detach_object.object.id = id;
    detach_object.link_name = end_effector_link_;
    detach_object.object.header.frame_id = control_->getPlanningFrame();
    detach_object.object.operation = moveit_msgs::CollisionObject::REMOVE;

    // Step 1: remove attached object
    planning_scene_interface_.applyAttachedCollisionObject(detach_object);

    // Step 2: remove collision object from scene
    removeCollision(id);

    return true;
  }

  bool MoveItClient::attachGazeboModel(const std::string& object_id, const std::string& link_name)
  {
    if (!initCheck()) return false;

    gazebo_ros_link_attacher::Attach attach_msg;
    attach_msg.request.model_name_1 = "robot";
    attach_msg.request.link_name_1 = "link_wrist_yaw";

    attach_msg.request.model_name_2 = object_id;
    attach_msg.request.link_name_2 = link_name;

    // Call attach service
    if (attach_gz_client_.call(attach_msg))
    {
      ROS_INFO_STREAM(__func__ << ": Model " << object_id << " attached!");
      return true;
    }
    else
    {
      ROS_ERROR_STREAM(__func__ << ": Failed to call attach service");
      return false;
    }
  }

  bool MoveItClient::detachGazeboModel(const std::string& object_id, const std::string& link_name)
  {
    if (!initCheck()) return false;

    gazebo_ros_link_attacher::Attach attach_msg;
    attach_msg.request.model_name_1 = "robot";
    attach_msg.request.link_name_1 = "link_wrist_yaw";

    attach_msg.request.model_name_2 = object_id;
    attach_msg.request.link_name_2 = link_name;

    // Call detach service
    if (detach_gz_client_.call(attach_msg))
    {
      ROS_INFO_STREAM(__func__ << ": Model " << object_id << " detached!");
      return true;
    }
    else
    {
      ROS_ERROR_STREAM(__func__ << ": Failed to call detach service");
      return false;
    }
  }

  std::optional<geometry_msgs::Pose> MoveItClient::getGazeboModelPose(const std::string& object_id)
  {
    gazebo_msgs::GetModelState model_state_msg;
    model_state_msg.request.model_name = object_id;

    // Call get gazebo model service
    if (gz_get_client_.call(model_state_msg))
    {
      if (model_state_msg.response.success)
      {
        ROS_INFO_STREAM(__func__ << ": Model " << object_id << " pose obtained!");
        return {model_state_msg.response.pose};
      }
      else
      {
        ROS_ERROR_STREAM(__func__ << ": Failed to find model "
            << object_id);
        return std::nullopt;
      }
    }
    else
    {
      ROS_ERROR_STREAM(__func__ << ": Failed to call Gazebo get models service");
      return std::nullopt;
    }
  }

  bool MoveItClient::setGazeboModelPose(
      const std::string& object_id,
      const geometry_msgs::Pose& pose,
      const std::string& ref_frame
    )
  {
    gazebo_msgs::SetModelState mode_state_msg;
    mode_state_msg.request.model_state.model_name = object_id;
    mode_state_msg.request.model_state.pose = pose;
    mode_state_msg.request.model_state.reference_frame = ref_frame;

    // Call set gazebo model service
    if (gz_set_client_.call(mode_state_msg))
    {
      if (mode_state_msg.response.success)
      {
        ROS_INFO_STREAM(__func__ << ": Setting model " << object_id << " pose!");
        return true;
      }
      else
      {
        ROS_ERROR_STREAM(__func__ << ": Failed to set model "
            << object_id << " pose!");
        return false;
      }
    }
    else
    {
      ROS_ERROR_STREAM(__func__ << ": Failed to call Gazebo set models service");
      return false;
    }
  }

  std::optional<geometry_msgs::PoseStamped> MoveItClient::getPoseInGivenFrame(
      const std::string& target_frame,
      const geometry_msgs::PoseStamped& pose
      )
  {
    if (!initCheck()) return std::nullopt;

    geometry_msgs::PoseStamped pose_in_target_frame;
    try
    {
      auto transform = tf_buffer_->lookupTransform(
          target_frame, pose.header.frame_id, ros::Time(0));

      tf2::doTransform(pose, pose_in_target_frame, transform);
    }
    catch (const std::exception &e)
    {
      ROS_WARN_STREAM(__func__ << ": Failed to tranform pose to given frame.");
      return std::nullopt;
    };
    return {pose_in_target_frame};
  }

  std::optional<geometry_msgs::PoseStamped> MoveItClient::getPoseInPlanningFrame(const geometry_msgs::PoseStamped& pose)
  {
    return getPoseInGivenFrame(control_->getPlanningFrame(), pose);
  }

  Eigen::Isometry3d MoveItClient::poseMsgToEigen(const geometry_msgs::Pose& msg)
  {
    Eigen::Isometry3d approx_target;
    Eigen::Translation3d translation(msg.position.x, msg.position.y, msg.position.z);
    Eigen::Quaterniond quaternion(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z);
    if ((quaternion.x() == 0) && (quaternion.y() == 0) && (quaternion.z() == 0) && (quaternion.w() == 0))
    {
      ROS_WARN_STREAM(__func__ << ": Empty quaternion found in pose message. Setting to neutral orientation.");
      quaternion.setIdentity();
    }
    else
    {
      quaternion.normalize();
    }
    approx_target = translation * quaternion;
    return approx_target;
  }

  bool MoveItClient::hoverArm(const geometry_msgs::PoseStamped& target_pose)
  {
    if (!initCheck()) return false;

    std::cout << "target pose: " << target_pose.pose << std::endl;

    auto transformed_pose = getPoseInPlanningFrame(target_pose);
    if (!transformed_pose.has_value()) return false;

    transformed_pose->pose.position.z += 0.25;
    std::cout << "transformed pose: " << transformed_pose->pose << std::endl;

    // movePoseWithConstraints(transformed_pose->pose.position.x, transformed_pose->pose.position.y, transformed_pose->pose.position.z);

    auto tries = 0;
    double dist;
    do
    {
      auto eef = control_->getCurrentPose(end_effector_link_);
      dist = std::hypot(eef.pose.position.x - transformed_pose->pose.position.x, eef.pose.position.y - transformed_pose->pose.position.y);
      std::cout << "dist: " << dist << std::endl;

      auto joint_values = control_->getCurrentJointValues();
      dist += joint_values[1];
      dist += joint_values[2];
      dist += joint_values[3];
      dist += joint_values[4];

      std::array<double, 4> arm_values;
      for (auto &v : arm_values)
      {
        if (dist >= 0.13)
        {
          v = 0.13;
        }
        else
        {
          v = std::max(0.0, dist);
        }
        dist -= v;
        std::cout << "dist: " << dist << std::endl;
      }
      joint_values = {transformed_pose->pose.position.z, arm_values[0], arm_values[1], arm_values[2], arm_values[3], -1.57};
      moveJoints(joint_values);
      // auto eef = control_->getCurrentPose(end_effector_link_);
      eef = control_->getCurrentPose(end_effector_link_);
      std::cout << "eef: " << eef << std::endl;
      dist = std::hypot(eef.pose.position.x - transformed_pose->pose.position.x, eef.pose.position.y - transformed_pose->pose.position.y);
      std::cout << "try #" << tries << std::endl;
      std::cout << "dist: " << dist << std::endl;
    } while ( dist > 0.05 && tries++ < 10);

    // control_->setApproximateJointValueTarget(transformed_pose->pose, end_effector_link_);
    //
    // double dist;
    // int tries = 0;
    // do
    // {
    //   if (moveit::planning_interface::MoveGroupInterface::Plan plan;
    //       control_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS)
    //   {
    //     control_->execute(plan);
    //   }
    //   else
    //   {
    //     ROS_ERROR_STREAM(__func__ << ": Failed to plan to target");
    //   }
      // auto eef = control_->getCurrentPose(end_effector_link_);
      // eef = control_->getCurrentPose(end_effector_link_);
      // std::cout << "eef: " << eef << std::endl;
      // dist = std::hypot(eef.pose.position.x - transformed_pose->pose.position.x, eef.pose.position.y - transformed_pose->pose.position.y);
      // std::cout << "try #" << tries << std::endl;
      // std::cout << "dist: " << dist << std::endl;
    // } while ( dist > 0.05 && tries++ < 10);

    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // return true;
    return (dist <= 0.05 ? true : false);
  }

  void MoveItClient::approachArm(const geometry_msgs::PoseStamped& target_pose)
  {
    if (!initCheck()) return;

    auto transformed_pose = getPoseInPlanningFrame(target_pose);
    if (!transformed_pose.has_value()) return;

    transformed_pose->pose.position.z += 0.05;
    // control_->setApproximateJointValueTarget(transformed_pose->pose, end_effector_link_);

    auto joint_values = control_->getCurrentJointValues();
    joint_values[0] = transformed_pose->pose.position.z;
    moveJoints(joint_values);

    // std::cout << "transformed pose: " << transformed_pose->pose << std::endl;

    // double dist;
    // int tries = 0;
    // do
    // {
    //   if (moveit::planning_interface::MoveGroupInterface::Plan plan;
    //       control_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS)
    //   {
    //     control_->execute(plan);
    //   }
    //   else
    //   {
    //     ROS_ERROR_STREAM(__func__ << ": Failed to plan to target");
    //   }
    //   auto eef = control_->getCurrentPose(end_effector_link_);
    //   // std::cout << "eef: " << eef << std::endl;
    //   dist = std::hypot(eef.pose.position.x - transformed_pose->pose.position.x, eef.pose.position.y - transformed_pose->pose.position.y);
    //   // std::cout << "try #" << tries << std::endl;
    //   // std::cout << "dist: " << dist << std::endl;
    // } while ( dist > 0.05 && tries++ < 10);

    // auto curr_pose = control_->getCurrentPose(end_effector_link_);
    // auto z = curr_pose.pose.position.z - 0.15;
    // moveZWithCurrentPose(z);

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }

  void MoveItClient::cancel()
  {
    control_->stop();
    gripper_control_->stop();
    camera_control_->stop();

    goPreset("home_low");
    goPresetGripper("open");
  }

  void MoveItClient::goPreset(const std::string& target)
  {
    if (!initCheck()) return;

    control_->setNamedTarget(target);

    if (moveit::planning_interface::MoveGroupInterface::Plan plan;
        control_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS)
    {
      control_->execute(plan);
    }
    else
    {
      ROS_ERROR_STREAM(__func__ << ": Failed to plan to target \"" << target << "\" positions." );
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }

  void MoveItClient::goPresetGripper(const std::string& target)
  {
    if (!initCheck()) return;

    gripper_control_->setNamedTarget(target);

    if (moveit::planning_interface::MoveGroupInterface::Plan plan;
        gripper_control_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS)
    {
      gripper_control_->execute(plan);
    }
    else
    {
      ROS_ERROR_STREAM(__func__ << ": Failed to plan gripper to preset \"" << target << "\"." );
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }

  void MoveItClient::moveJoints(const std::vector<double>& joint_group_positions)
  {
    if (!initCheck()) return;

    control_->setJointValueTarget(joint_group_positions);

    if (moveit::planning_interface::MoveGroupInterface::Plan plan;
        control_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS)
    {
      control_->execute(plan);
    }
    else
    {
      ROS_ERROR_STREAM(__func__ << ": Failed to plan to target joint group positions");
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }

  void MoveItClient::movePose(const std::vector<double>& pose)
  {
    if (!initCheck()) return;

    geometry_msgs::Pose target_pose;
    target_pose.position.x = pose[0];
    target_pose.position.y = pose[1];
    target_pose.position.z = pose[2];

    tf2::Quaternion quat;
    quat.setRPY(pose[3], pose[4], pose[5]);
    target_pose.orientation.x = quat.getX();
    target_pose.orientation.y = quat.getY();
    target_pose.orientation.z = quat.getZ();
    target_pose.orientation.w = quat.getW();

    control_->setStartStateToCurrentState();
    control_->setApproximateJointValueTarget(target_pose, end_effector_link_);

    if (moveit::planning_interface::MoveGroupInterface::Plan plan;
        control_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS)
    {
      control_->execute(plan);
    }
    else
    {
      ROS_ERROR_STREAM(__func__ << ": Failed to plan to target pose position");
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }

  void MoveItClient::moveRandomValidPose()
  {
    if (!initCheck()) return;

    control_->setStartStateToCurrentState();
    control_->setRandomTarget();

    if (moveit::planning_interface::MoveGroupInterface::Plan plan;
        control_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS)
    {
      control_->execute(plan);
    }
    else
    {
      ROS_ERROR_STREAM(__func__ << ": Failed to plan to random valid pose position");
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }

  void MoveItClient::moveZWithCurrentPose(const double z)
  {
    if (!initCheck()) return;

    auto curr_pose = control_->getCurrentPose(end_effector_link_);
    curr_pose.pose.position.z = z;

    control_->setApproximateJointValueTarget(curr_pose.pose, end_effector_link_);
    // Get robot current state
    auto curr_state = control_->getCurrentState();

    // Create joint value container
    std::vector<double> joint_values;

    if (curr_state->setFromIK(joint_model_group_, curr_pose.pose))
    {
      // Get resulting joint value
      curr_state->copyJointGroupPositions(joint_model_group_, joint_values);

      // Set these as target joint value
      control_->setJointValueTarget(joint_values);

      control_->setStartStateToCurrentState();

      if (moveit::planning_interface::MoveGroupInterface::Plan plan;
          control_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS)
      {
        control_->execute(plan);
      }
      else
      {
        ROS_ERROR_STREAM(__func__ << ": Failed to plan to z (" << z << ") with curr pose position");
      }
    }
    else
    {
      ROS_ERROR_STREAM(__func__ << ": Failed to find IK for given z (" << z << ") with curr pose position");
    }

    control_->setStartStateToCurrentState();

    if (moveit::planning_interface::MoveGroupInterface::Plan plan;
        control_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS)
    {
      control_->execute(plan);
    }
    else
    {
      ROS_ERROR_STREAM(__func__ << ": Failed to plan to z (" << z << ") with curr pose position");
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }

  void MoveItClient::movePoseWithConstraints(const double x, const double y, const double z)
  {
    if (!initCheck()) return;

    auto target_pose = control_->getCurrentPose(end_effector_link_);
    target_pose.pose.position.x = x;
    target_pose.pose.position.y = y;
    target_pose.pose.position.z = z;

    control_->setStartStateToCurrentState();
    control_->setApproximateJointValueTarget(target_pose, end_effector_link_);

    // Set constraint
    moveit_msgs::OrientationConstraint oc;
    oc.link_name = reference_frame_;
    oc.header.frame_id = end_effector_link_;
    oc.orientation = target_pose.pose.orientation;
    oc.absolute_x_axis_tolerance = 0.1;
    oc.absolute_y_axis_tolerance = 0.1;
    oc.absolute_z_axis_tolerance = 0.1;
    oc.weight = 1.0;

    // Set as path constraint for the group
    moveit_msgs::Constraints constraint;
    constraint.orientation_constraints.push_back(oc);

    control_->setPathConstraints(constraint);
    control_->setStartStateToCurrentState();
    control_->setApproximateJointValueTarget(target_pose, end_effector_link_);

    if (moveit::planning_interface::MoveGroupInterface::Plan plan;
        control_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS)
    {
      control_->execute(plan);
    }
    else
    {
      ROS_ERROR_STREAM(__func__ << ": Failed to plan to z (" << z << ") with curr pose position");
    }

    control_->clearPathConstraints();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }

  void MoveItClient::addBox(const std::vector<double>& pose, const std::string& id, const std::string& frame_id)
  {
    if (!initCheck()) return;

    if (planning_scene_pub_.getNumSubscribers() == 0u)
    {
      ROS_WARN_STREAM(__func__ << ": Planning scene not available, please try again later.");
      return;
    }

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit_msgs::PlanningScene planning_scene;
    moveit_msgs::CollisionObject collision_object;

    // Set id of the object
    collision_object.id = id;
    collision_object.header.frame_id = frame_id.empty() ? reference_frame_ : frame_id;

    // Define a box to add to the planning scene
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 2;
    primitive.dimensions[primitive.BOX_Y] = 2;
    primitive.dimensions[primitive.BOX_Z] = 0.01;

    // Define a pose for box
    geometry_msgs::Pose target_pose;
    target_pose.position.x = pose[0];
    target_pose.position.y = pose[0];
    target_pose.position.z = pose[0];

    tf2::Quaternion quat;
    quat.setRPY(pose[3], pose[4], pose[5]);
    target_pose.orientation.x = quat.getX();
    target_pose.orientation.y = quat.getY();
    target_pose.orientation.z = quat.getZ();
    target_pose.orientation.w = quat.getW();

    collision_object.header.frame_id = control_->getPlanningFrame();
    collision_object.operation = collision_object.ADD;
    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(target_pose);

    planning_scene.world.collision_objects.push_back(collision_object);
    planning_scene.is_diff = true;

    planning_scene_pub_.publish(planning_scene);

    ROS_INFO_STREAM(__func__ << ": Added a box into the world with id " << id << ".");
  }

  void MoveItClient::removeBox(const std::string& id)
  {
    if (!initCheck()) return;

    // Define the collision object
    moveit_msgs::CollisionObject collision_object;
    collision_object.id = id;
    collision_object.header.frame_id = control_->getPlanningFrame();
    collision_object.operation = moveit_msgs::CollisionObject::REMOVE;

    planning_scene_interface_.applyCollisionObject(collision_object);

    ROS_INFO_STREAM(__func__ << ": Removed object id: " << id << ".");
  }

  void MoveItClient::printUsefulInfo()
  {
    if (!initCheck()) return;

    // Print current pose of end effector
    auto curr_pose = control_->getCurrentPose(end_effector_link_);

    ROS_INFO_STREAM(
        "Current Pose (EEF) p.x:" << curr_pose.pose.position.x
        << ", p.y: " << curr_pose.pose.position.y
        << ", p.z: " << curr_pose.pose.position.z
        << ", q.x: " << curr_pose.pose.orientation.x
        << ", q.y: " << curr_pose.pose.orientation.y
        << ", q.z: " << curr_pose.pose.orientation.z
        << ", q.w: " << curr_pose.pose.orientation.w
      );

    // Print current joint values
    auto curr_joint = control_->getCurrentJointValues();
    auto joint_name = control_->getVariableNames();
    std::stringstream ss;

    std::transform(joint_name.begin(), joint_name.end(),
        curr_joint.begin(), curr_joint.begin(),
        [&ss](const std::string& name, const double val)
        {
          ss << name << ": " << val << ", ";
          return val;
        }
      );

    ROS_INFO_STREAM("Current Joint Values:" << ss.str());

    // Print current RPY of end effector
    auto rpy = control_->getCurrentRPY(end_effector_link_);

    ROS_INFO_STREAM(
        "Current RPY (EEF) roll:" << rpy[0]
        << ", pitch:" << rpy[1]
        << ", yaw:" << rpy[2]
      );

    // Print planner id
    ROS_INFO_STREAM("Current planner: " << control_->getPlannerId());
  }
} // moveit_control
