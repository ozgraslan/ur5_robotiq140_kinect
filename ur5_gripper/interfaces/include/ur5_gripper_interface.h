#ifndef INTERFACES_UR5_GRIPPER_INTERFACE_H
#define INTERFACES_UR5_GRIPPER_INTERFACE_H

#include <pluginlib/class_loader.h>
#include <ros/ros.h>

// MoveIt
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <boost/scoped_ptr.hpp>

class UR5GripperInterface{

private:
    moveit::planning_interface::MoveGroupInterface arm_group;
    moveit::planning_interface::MoveGroupInterface gripper_group;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    const moveit::core::JointModelGroup* joint_model_group;
public:
    UR5GripperInterface(std::string arm_group_name, std::string gripper_group_name);
    ~UR5GripperInterface();
    //pose w, x, y, z
    bool go_pose(std::vector<float> pose);

}




#endif // INTERFACES_UR5_GRIPPER_INTERFACE_H