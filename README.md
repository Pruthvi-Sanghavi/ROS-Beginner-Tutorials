# ROS Publisher Subscriber (ROS Tf, Unit Testing and Bag files) Beginner Tutorials
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

## Overview
This tutorial deals with tf, rosbag and unit testing in ROS

## Dependencies

 Ubuntu 16.04 Xenial ([link](http://releases.ubuntu.com/16.04/))
 ROS kinetic ([link](http://wiki.ros.org/kinetic))
 Catkin ([link](http://wiki.ros.org/catkin))  


## Install ROS

```
Open a terminal
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
$ sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
$ sudo apt-get update
$ sudo apt-get install ros-kinetic-desktop-full
$ sudo rosdep init
$ rosdep update
$ echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
$ source ~/.bashrc
$ source /opt/ros/kinetic/setup.bash
$ sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential
```

## Building Workspace and Packages
Use the following commands on the terminal to build the workspace and the package with the relevant dependencies.

```
Open a terminal
$ mkdir -p ~/catkin_ws
$ cd catkin_ws
$ mkdir src
$ catkin_make
$ source devel/setup.bash
$ cd src
$ git clone https://github.com/Pruthvi-Sanghavi/beginner_tutorials.git
$ cd ..
$ catkin_make
$ source devel/setup.bash 
```

## Run Publisher and Subscriber nodes

Code for publisher and subscriber are available at [link](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29)
Code for Debug is available at [link](http://wiki.ros.org/ROS/Tutorials/ExaminingPublisherSubscriber)

```
Open Terminal - 1
$ roscore

Open Terminal - 2
$ cd catkin_ws
$ source devel/setup.bash
$ rosrun beginner_tutorials talker <frequency>


Open Terminal - 3
$ cd catkin_ws
$ source devel/setup.bash
$ rosrun beginner_tutorials listener

```

## Using Launch
The roslaunch command starts the nodes. Along with that it also begins the ros master.
```
Open a terminal
$ cd catkin_ws
$ source devel/setup.bash
$ roslaunch beginner_tutorials hw11.launch

To terminate
Press Ctrl+C.
```

## ROS Service
To change the string output replace "Changed String" to something.
```
Open a terminal
$ cd catkin_ws
$ source devel/setup.bash
$ roslaunch beginner_tutorials hw11.launch

Open a Terminal
$ cd catkin_ws
$ source devel/setup.bash
$ rosservice call /change_string_output "Changed String"
```

## ROS Logging

```
Open a Terminal
$ cd catkin_ws
$ source devel/setup.bash
$ roslaunch beginner_tutorials hw11.launch

In another terminal
$ rqt_console

In another terminal
$ rqt_logger_level
```

## tf Frame verification

### Using rqt_tf_tree
```
Open a terminal
$ cd catkin_ws
$ source devel/setup.bash
$ roslaunch beginner_tutorials hw11.launch

Open another terminal
$ cd catkin_ws
$ source devel/setup.bash
$ rosrun rqt_tf_tree rqt_tf_tree
```

### Using tf_echo
```
Open a terminal

$ cd catkin_ws
$ source devel/setup.bash
$ rosrun tf tf_echo world talk
```

### Using view_frames
```
Open a terminal
$ cd catkin_ws
$ rosrun tf view_frames
```

## ROS Unit tests
Run the unit testing in the new terminal.
Open a terminal
### Testing using Launch
```
$ cd catkin_ws
$ source devel/setup.bash
$ rostest beginner_tutorials unitTest.launch

```
### Testing using catkin_make
```
$ cd ~/catkin_ws
$ source devel/setup.bash
$ catkin_make run_tests beginner_tutorials
```

## Playing and Recording Bag files
Bag files are used to record data and play data as and when required.
To enable or disable the rosbag record and play option enter the launch directory and then launch file and set the default argument to true or false respectively.
### Recording bag files
After launching the nodes

Open terminal 1
```
$ rosscore
```

Open terminal 2
```
$ cd catkin_ws
$ source devel/setup.bash
$ rosrun beginner_tutorials talker
```
Open terminal 3
```
$ cd catkin_ws
$ source devel/setup.bash
$ cd src/beginner_tutorials/Results
$ rosbag record -a

ctrl + c after 15 seconds to terminate
```

### Examining Bag files
In the same terminal used in recording bag file, type the following to examine the bag file.

```
$ rosbag info record.bag
```

### Playing the bag file

First we will run the listener node.

Open terminal 1
```
$ cd catkin_ws
$ source devel/setup.bash
$ rosrun beginner_tutorials listener
```
Now will play the recorded talker bag file
Open terminal 2
```
$ cd catkin_ws
$ source devel/setup.bash
$ cd src/beginner_tutorials/Results
$ rosbag play record.bag
```

## Google Styling

Google Styling Results are given
```
cpplint and cppcheck
$ roscd beginner_tutorials
$ cd Results
```

