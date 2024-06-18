# Setup Description




# Installation

This installation is designed for running on **Ubuntu** operating systems. While this setup might work on other operating systems, we recommend running this setup on Ubuntu 20.04.6.

## 1) Install Python 3.8.10

Python 3.8.10 can be installed from the Python website: *https://www.python.org/downloads/release/python-3810/*

While our configuration will probably work on other versions, this is the version that we employed for controlling our robots

## 2) Install ROS Noetic

Next, it is important to install the Robot Operating System (ROS) package. This is the underlying framework that is utilised by MoveIt to communicate with the UR5 robot effectively. 

Ros Noetic can be installed from the ROS website: *http://wiki.ros.org/noetic/Installation/Ubuntu*

We have not tested our setup with any other version of ROS, while other versions may work, we recommend using Noetic.

Remember to source the noetic setup filepath and add this to your bashrc file (to ensure these files are sourced on launch). Example: **echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc**

## 3) Configure Catkin workspace

It is important to also install Catkin, which manages ROS packages and is used for creating a workspace. If this has not already been installed, then install via the following command: **sudo apt-install python3-catkin-tools ros-noetic-catkin**

*If you already have a catkin workspace, then skip this part.*

Next, create a catkin workspace at the top-level directory: **mkdir -p ~/catkin_ws/src**
**cd ~/catkin_ws/src**
**rosdep install -y --from-paths . --ignore-src --rosdistro melodic**
**cd ~/catkin_ws**
**catkin config --extend /opt/ros/${ROS_DISTRO} --cmake-args -DCMAKE_BUILD_TYPE=Release**
**catkin build**

Remember to source the catkin setup filepath and add this to your bashrc file: **echo "source ~/catkin_ws/devel/setup.bach" >> ~/.bashrc**

The src directory will contain the ROS packages needed for controlling the UR5. These will need to be built, using catkin, in order to the executed. 

For more info, please visit: *http://wiki.ros.org/catkin/workspaces*

## 4) Install MoveIt Noetic

MoveIt is the framework that can control a variety of different robots. 

MoveIt can be installed from the MoveIt website: *https://moveit.ros.org/install/* 
Or by executing the following command: **sudo apt install ros-noetic-moveit**

## 5) Install TRAC-IK Kinematics Solver

We utilised the Trac-IK kinematric solver for calculating the path planning for the robots. This needs to be installed seperately to function.

TRAC-IK Kinematics Solver can be installed on the ROS website: *https://docs.ros.org/en/kinetic/api/moveit_tutorials/html/doc/trac_ik/trac_ik_tutorial.html* 
Or by executing the following command: **sudo apt-get install ros-kinetic-trac-ik-kinematics-plugin**

## 6) Install Gazebo

Gazebo is a software package that can simulate robots virtually. If you plan on only using these config files on a real UR5, this can be skipped.

Gazebo can be installed via the Gazebo website: https://classic.gazebosim.org/tutorials?tut=install_ubuntu

## 7) Install Universal Robot MoveIt package

In order to control the Ur5 robots, the universal robot moveit package must be installed. These contain all the config, controller and kinematic files required for running the robots.

Make sure that you are in the correct directory: **cd ~/catkin_ws/src**
Next, use clone the repo: **git clone https://github.com/ros-industrial/universal_robot**
Make sure to rebuild: **cd ~/catkin_ws**
**catkin build**

## 8) Install Universal Robot Driver package 

In order to communicate with the UR5, the robot driver package needs to be installed.

Make sure that you are in the correct directory: **cd ~/catkin_ws/src**
Next, use clone the repo: **git clone https://github.com/UniversalRobots/Universal_Robots_ROS_Driver.git**
Make sure to rebuild: **cd ~/catkin_ws**
**catkin build** 

## 9) Install URCAP on robot:

https://github.com/UniversalRobots/Universal_Robots_ExternalControl_URCap/releases

# Customisations




