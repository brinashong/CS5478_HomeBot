#pragma once

// ROS
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

// MoveIt
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <string>
#include <tf2/LinearMath/Quaternion.h>

// Gazebo
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_ros_link_attacher/Attach.h>


// STL
#include <memory>
#include <vector>
#include <optional>
#include <chrono>
#include <thread>
#include <algorithm>
#include <sstream>

namespace moveit_control
{
  class MoveItClient
  {
  public:
    explicit MoveItClient(const ros::NodeHandle& n,
        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> control,
        const std::string& planning_group,
        std::vector<moveit_msgs::JointConstraint> default_joint_contraints = {});

    bool addCollision(const moveit_msgs::CollisionObject& collision_object);

    bool addCollisions(const std::vector<moveit_msgs::CollisionObject>& collision_objects);

    bool removeCollision(const std::string& id);

    bool clearCollisions();

    bool attachObject(const std::string& _object_name,
        const moveit_msgs::AttachedCollisionObject& _attached_object,
        const std::string& _group_name
      );

    bool detachObject(const std::string& id);

    /**
     * \brief Attach Gazebo model to end effector
     */
    bool attachGazeboModel(const std::string& object_id);

    /**
     * \brief Detach Gazebo model from end effector
     */
    bool detachGazeboModel(const std::string& object_id);

    /**
     * \brief Get Gazebo model pose
     */
    std::optional<geometry_msgs::Pose> getGazeboModelPose(const std::string& object_id);

    /**
     * \brief Set Gazebo model pose
     */
    bool setGazeboModelPose(
        const std::string& object_id,
        const geometry_msgs::Pose& pose,
        const std::string& ref_frame
      );

    void goPreset(const std::string& target);

    void moveJoints(const std::vector<double>& joint_group_positions);

    /**
     * \brief move to pose
     *
     * \param pose {x, y, z, roll, pitch, yaw}
     */
    void movePose(const std::vector<double>& pose);

    /**
     * \brief move to random valid pose
     */
    void moveRandomValidPose();

    /**
     * \brief move only z axis while keeping current pose
     *
     * \param z axis value
     */
    void moveZWithCurrentPose(const double z);

    /**
     * \brief move to pose with end effector link
     *  constrained to its current pose
     *
     * \param x axis value
     * \param y axis value
     * \param z axis value
     */
    void movePoseWithConstraints(const double x, const double y, const double z);

    /**
     * \brief add box in planning scene with fix dimention
     *  {x, y, z} {2, 2, 0.01}
     *
     * \param pose {x, y, z, roll, pitch, yaw}
     * \param id of collision object
     * \param frame_id frame for box, otherwise reference_frame_ used
     */
    void addBox(const std::vector<double>& pose,
        const std::string& id, const std::string& frame_id);

    /**
     * \brief remove box from planning scene
     *
     * \param id of collision object to be removed
     */
    void removeBox(const std::string& id);

    /**
     * \brief print useful information about current moveit server
     */
    void printUsefulInfo();

  private:
    bool initCheck();
    ros::NodeHandle n_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> control_;
    ros::Publisher planning_scene_pub_;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
    std::vector<moveit_msgs::JointConstraint> default_joint_contraints_;

    ros::ServiceClient gz_get_client_;
    ros::ServiceClient gz_set_client_;
    ros::ServiceClient attach_gz_client_;
    ros::ServiceClient detach_gz_client_;

    bool initialize_;
    double position_tolerance_;
    double orientation_tolerance_;
    double joint_tolerance_;

    double max_acc_scale_factor_;
    double max_vel_scale_factor_;
    unsigned int num_planning_attempt_;

    std::string end_effector_link_;
    std::string reference_frame_;

    double planning_time_;
    const robot_state::JointModelGroup* joint_model_group_;
  };
} // moveit_control