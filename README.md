# ur5_robotiq140_kinect

Works on Ubuntu 18.04 and ROs-Melodic.

How to install:
    * Install ROS-Melodic from: http://wiki.ros.org/melodic/Installation/Ubuntu
    * First install moveit from: https://ros-planning.github.io/moveit_tutorials/doc/getting_started/getting_started.html
    * Create a new catkin workspace with src directory
    * Clone repositories into src directory: 
        - git clone -b kinetic-devel https://github.com/ros-industrial/universal_robot.git
        - git clone https://github.com/JenniferBuehler/general-message-pkgs.git
        - git clone https://github.com/ozgraslan/ur5_robotiq140_kinect.git
        - git clone https://github.com/RobotRose/kinect_description.git
        - git clone https://github.com/ozgraslan/gazebo-pkgs.git
        - git clone https://github.com/roboticsgroup/roboticsgroup_gazebo_plugins.git
        - git clone https://github.com/ozgraslan/robotiq.git
    * Before build: source  ~/<ws_moveit>/devel/setup.bash
    * Build the packages


How to use:
    * Launch gazebo with robot: roslaunch ur5_gripper ur5_gripper.launch [limited:=true]
    * Launch moveit: roslaunch ur5_gripper_moveit_config ur5_gripper_moveit_planning_execution.launch sim:=true [limited:=true]
    * Launch rviz: roslaunch ur5_gripper_moveit_config moveit_rviz.launch config:=true
