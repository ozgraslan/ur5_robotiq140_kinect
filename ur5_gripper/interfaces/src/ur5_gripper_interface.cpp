// for group names check ur5.srdf
// ur5_arm: group name for ur5
// robotiq140_gripper: group name for robotiq_2f_140 

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <ur5_gripper_interface.h>

UR5GripperInterface::UR5GripperInterface(std::string arm_group_name, std::string gripper_group_name) : 
arm_group(arm_group_name), gripper_group(gripper_group_name), joint_model_group(move_group.getCurrentState()->getJointModelGroup(arm_group_name)){
    ROS_INFO_NAMED("ur_robotiq140", "Planning frame: %s", move_group.getPlanningFrame().c_str());
    ROS_INFO_NAMED("ur_robotiq140", "End effector link: %s", move_group.getEndEffectorLink().c_str());
    ROS_INFO_NAMED("ur_robotiq140", "Available Planning Groups:");
    std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),std::ostream_iterator<std::string>(std::cout, ", "));
}

bool UR5GripperInterface::go_pose(std::vector<float> pose){
    geometry_msgs::Pose target_pose;
    target_pose.orientation.w = pose[0];
    target_pose.position.x = pose[1];
    target_pose.position.y = pose[2];
    target_pose.position.z = pose[3];
    self -> arm_group.setPoseTarget(target_pose);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("ur_robotiq140", "Visualizing plan (pose goal) %s", success ? "" : "FAILED");
    if(success){
        ROS_INFO_NAMED("ur_robotiq140", "Visualizing plan as trajectory line");
        visual_tools.publishTrajectoryLine(plan.trajectory_, joint_model_group);
        visual_tools.trigger();
    }
}

int main(int argc char** argv){
    ros::init(argc, argv, "ur5_cpp_interface");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    UR5GripperInterface test(arm_group_name="ur5_arm", gripper_group_name="robotiq140_gripper");
    return 0;
}