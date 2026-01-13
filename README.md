NNOFarmsAI

This repository contains a ROS 2 (Humble) workspace for simulating a UR5 pick-and-place task using Gazebo and MoveIt 2.

The project focuses on motion planning, collision handling, and execution robustness rather than gripper control or perception.

Workspace Structure
sim_ws/
├── src/
│   ├── Universal_Robots_ROS2_Gazebo_Simulation   (git submodule)
│   └── pick_place_cpp
│       ├── src/pick_place.cpp
│       ├── CMakeLists.txt
│       └── package.xml
├── .gitmodules
├── .gitignore
└── README.md

System Requirements

Ubuntu 22.04

ROS 2 Humble

Gazebo (gazebo_ros)

MoveIt 2

Dependencies Installation
sudo apt update
sudo apt install -y \
  ros-humble-moveit \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-ur \
  ros-humble-ur-robot-driver


Source ROS:

source /opt/ros/humble/setup.bash

Clone and Setup
git clone https://github.com/Vamshi430/INNOFarmsAI.git
cd INNOFarmsAI
git submodule update --init --recursive

Build
cd sim_ws
colcon build --symlink-install
source install/setup.bash

Run Simulation
Launch UR5 in Gazebo
ros2 launch ur_simulation_gazebo ur_sim_moveit.launch.py ur_type:=ur5


Run Pick and Place Node
ros2 run pick_place_cpp pick_place

Behavior Description

Robot moves to home position

Approaches the object from above

Performs Cartesian descend

Attaches object logically using MoveIt

Lifts and places object at target location

Detaches object and returns home

No physical gripper is used; object attachment is handled in the planning scene.

Known Issues / Limitations

Cartesian path planning may fail in some configurations

No perception or force feedback

Kinematics warnings may appear but planning still works

These limitations are expected and documented.

Notes

This project demonstrates:

ROS 2 + MoveIt integration

Planning scene usage

Cartesian and joint-space planning

Debugging of kinematics and execution issues

Author

Vamshi
