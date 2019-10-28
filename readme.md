# ROS Publisher Subscriber Beginner Tutorials

## Overview
ROS is an opensource robotics operating system developed and maintained by Open Source Robotics Foundation 
This tutorial familiarizes with the ros graph and its elements such as rosnode, roscore, ros messages, ros master.
The project beginner_tutorials is about creating publisher and subscriber nodes and transferring messages between them.

## Install ROS

```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt-get update
sudo apt-get install ros-kinetic-desktop-full
sudo rosdep init
rosdep update
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
source /opt/ros/kinetic/setup.bash
sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential
```

### Building Workspace and Packages

```
$mkdir -p ~/catkin_ws
$cd catkin_ws
$mkdir src
$catkin_make
$source devel/setup.bash
$cd src
$catkin_create_pkg beginner_tutorials roscpp rospy std_msgs
$cd ..
$catkin_make
$source devel/setup.bash
```
### Run Publisher and Subscriber

Code for publisher is available at [link](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29)
Code for subscriber is available at [link](http://wiki.ros.org/ROS/Tutorials/ExaminingPublisherSubscriber)

```
$roscore

Open new terminal

$rosrun beginner_tutorials talker.cpp
$rosrun beginner_tutorials listener.cpp
```
## Google Styling
### cppcheck
Open cppCheck in the Packages
```
$cd catkin_ws/src/beginner_tutorials/gedit cppCheck.cpp
```
### cpplint
```
$cd catkin_ws/src/beginner_tutorials/gedit cpplint.cpp
```
