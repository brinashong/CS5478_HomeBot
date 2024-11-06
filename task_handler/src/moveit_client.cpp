#include "task_handler/moveit_client.hpp"
#include "moveit_msgs/Constraints.h"
#include "moveit_msgs/OrientationConstraint.h"

namespace moveit_control
{
  MoveItClient::MoveItClient(const ros::NodeHandle& n,
      std::shared_ptr<moveit::planning_interface::MoveGroupInterface> control,
      const std::string& planning_group,
      std::vector<moveit_msgs::JointConstraint> default_joint_contraints)
    : n_{n}
    , control_{control}
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
  {
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

    // // Create joint value container
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
