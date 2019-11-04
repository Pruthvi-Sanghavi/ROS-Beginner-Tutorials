# ROS Publisher Subscriber Beginner Tutorials
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

## Overview
ROS is an opensource robotics operating system developed and maintained by Open Source Robotics Foundation 
This tutorial familiarizes with the ros graph and its elements such as rosnode, rostopic, ros messages, ros master, ros services.
The project beginner_tutorials is about creating publisher and subscriber nodes and transferring messages between them.

## Dependencies

 Ubuntu 16.04 Xenial ([link](http://releases.ubuntu.com/16.04/))
 ROS kinetic ([link](http://wiki.ros.org/kinetic))
 Catkin ([link](http://wiki.ros.org/catkin))  
 

## Install ROS

```
Open a terminal
~$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
~$ sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
~$ sudo apt-get update
~$ sudo apt-get install ros-kinetic-desktop-full
~$ sudo rosdep init
~$ rosdep update
~$ echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
~$ source ~/.bashrc
~$ source /opt/ros/kinetic/setup.bash
~$ sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential
```

## Building Workspace and Packages

```
Open a terminal
~$ mkdir -p ~/catkin_ws
~$ cd catkin_ws
~$ mkdir src
~$ catkin_make
~$ source devel/setup.bash
~$ cd src
~$ git clone https://github.com/Pruthvi-Sanghavi/beginner_tutorials.git
~$ cd ..
~$ catkin_make
~$ source devel/setup.bash 
```

## Run Publisher and Subscriber nodes

Code for publisher and subscriber are available at [link](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29)
Code for Debug is available at [link](http://wiki.ros.org/ROS/Tutorials/ExaminingPublisherSubscriber)

```
Open Terminal - 1
~$ roscore

Open Terminal - 2
~$ cd catkin_ws
~$ rosrun beginner_tutorials talker <frequency>

Open Terminal - 3
~$ rosrun beginner_tutorials listener

```

## Using Launch

```
Open a terminal
~$ cd catkin_ws
~$ source devel/setup.bash
~$ roslaunch --screen beginner_tutorials Week10_HW.launch

To terminate
Press Ctrl+C.
```

## Google Styling

### cppcheck

```


```
### cpplint
```


```

