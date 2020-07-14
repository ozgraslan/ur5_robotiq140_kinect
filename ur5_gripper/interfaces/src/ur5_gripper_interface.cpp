// for group names check ur5.srdf
// ur5_arm: group name for ur5
// robotiq140_gripper: group name for robotiq_2f_140 

#include <ur5_gripper_interface.h>

UR5GripperInterface::UR5GripperInterface(std::string arm_group_name, std::string gripper_group_name) : 
arm_group(arm_group_name), gripper_group(gripper_group_name), joint_model_group(move_group.getCurrentState()->getJointModelGroup(arm_group_name)){
    ROS_INFO_NAMED("ur_robotiq140", "Planning frame: %s", move_group.getPlanningFrame().c_str());
    ROS_INFO_NAMED("ur_robotiq140", "End effector link: %s", move_group.getEndEffectorLink().c_str());
    ROS_INFO_NAMED("ur_robotiq140", "Available Planning Groups:");
    std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),std::ostream_iterator<std::string>(std::cout, ", "));
}

UR5GripperInterface::go_pose(std::vector<float> pose){
    geometry_msgs::Pose target_pose;
    target_pose1.orientation.w = pose[0]
    target_pose1.position.x = pose[1]
    target_pose1.position.y = pose[2]
    target_pose1.position.z = pose[3]
    self -> arm_group.setPoseTarget(target_pose1);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("ur_robotiq140", "Visualizing plan (pose goal) %s", success ? "" : "FAILED");
    if(success){
        ROS_INFO_NAMED("ur_robotiq140", "Visualizing plan as trajectory line");
        visual_tools.publishTrajectoryLine(plan.trajectory_, joint_model_group);
        visual_tools.trigger();
    }
}