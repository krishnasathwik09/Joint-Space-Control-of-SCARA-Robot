# Joint-Space-Control-of-SCARA-Robot







## Description
This package simulates a joint space position control for a 3-DOF revolute-revolute- prismatic (RRP) robot manipulator in **Gazebo**.

The robot joints are controlled using an independent joint control framework, that is, each joint of the robot is controlled via a separate PID controller.
The control approach is considered a position-based control method, that is, the desired set-points provided to the controller are (xd, yd, zd) coordinates of the end-effector.

**Part 1.** 

Implemented an inverse kinematics node as a service server that takes a (desired) pose of the end effector and returns joint configurations as the response.

**Part 2.**

1.Subscribe to the /rrpbot/joint_states topic to obtain joints’ positions and velocities in real-time.

2.Utilized a full PID control framework to calculate the required joint torques to move the robot to the desired set-point.

3.Applied the joint torques/forces calculated by the controller to the robot by publishing torque/force commands to the following topics:

                /rrpbot/joint1_effort_controller/command
                
                /rrpbot/joint2_effort_controller/command
                
                /rrpbot/joint3_effort_controller/command
                
**Performance Constraints:**
a.No overshoot and oscillations

b.Joint effort should be with in Joint Limits

c.Suppression of Coupling effects due to joints

d.Task Completion under 1 min

## Dependecies:
This repository has been developed and tested in Ubuntu 16.04 and ROS Kinetic.

#### Set up the RRP robot in Gazebo

• First, download the ROS package for the RRP robot manipulator from the this repositry, and place it under your ROS workspace:

`cd ~/rbe500_ros/src`

• We can install the dependencies by the following commands:

`sudo apt-get install ros-kinetic-ros-control ros-kinetic-ros-controllers`

`sudo apt-get install ros-kinetic-gazebo-ros-pkgs ros-kinetic-gazebo-roscontrol`

• With all dependencies ready, build the ROS package by:

`cd ~/rbe500_ros`

`catkin_make`


## Setup and Run
i) Open a terminal and launch the RRP robot manipulator in Gazebo before running the scripts.

  `roslaunch rrp_gazebo gazebo.launch`
  
ii) Once the robot is successfully spawned in Gazebo, open a new terminal and launch the effort controller node and the joint state publisher by using the command:

  `roslaunch rrp_control rrp_effort_control.launch`
  
iii) We can now test the inverse kinematics and position control scripts by running the rrp.launch file in another terminal:

  `roslaunch rbe500_project rrp.launch`






