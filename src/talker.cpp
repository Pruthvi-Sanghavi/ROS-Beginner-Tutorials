/**Copyright <2019> <Pruthvikumar Sureshkumar Sanghavi>
 *
 *Permission is hereby granted, free of charge, to any person obtaining a copy of this 
 *software and associated documentation files (the "Software"), to deal in the Software 
 *without restriction, including without limitation the rights to use, copy, modify, merge, 
 *publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons 
 *to whom the Software is furnished to do so, subject to the following conditions:
 *
 *The above copyright notice and this permission notice shall be included in all copies or substantial 
 *portions of the Software.
 *
 *THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT 
 *LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN 
 *NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, 
 *WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE 
 *OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */
/**
 * @file 		  talker.cpp
 * @author 		Pruthvikumar Sanghavi
 * @copyright MIT
 * @brief 		ROS Publisher
 */

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>
#include <tf/transform_broadcaster.h>
#include "beginner_tutorials/change_string_output.h"

std::string text = " Hallo, Ich bin Pruthvi!!! ";
extern std::string text;

/**
 * @brief a function that changes the stream message
 * @param req represents the data being sent to the service
 * @param res represents the data being provided by the client
 * @return bool
 */
bool newtext(beginner_tutorials::change_string_output::Request &req,
                beginner_tutorials::change_string_output::Response &res) {
  text = req.originaltext;
  ROS_INFO_STREAM("The string is changed to the new string: ");
  res.newtext = req.originaltext;
  return true;
}
/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv) {
  /**
   * The ros::init() function initializes the talker node
   */ 
  ros::init(argc, argv, "talker");

  // Using a variable to store freq
  int freq = 2;

  /**
   * NodeHandle maintains the communication with ROS.
   */
  ros::NodeHandle n;

  ros::Rate loop_rate(freq);
  // Let's generate log messages
  if (argc > 1) {
    // Converting string argument to integer
    freq = atoi(argv[1]);

    if (freq < 0) {
      // For negative frequency or less than zero
      ROS_FATAL_STREAM("Frequency cannot be less than zero");
      ros::shutdown();
      return 1;
    } else if (freq == 0) {
      // For frequency equals to '0'
      ROS_ERROR_STREAM("Frequency cannot be zero!!");
      ROS_INFO_STREAM("Changing frequency to 4hz");
      freq = 4;
    } else if (freq >= 10) {
      // For frequency greater than 10
      ROS_WARN_STREAM("very fast to read!!");
      ROS_INFO_STREAM("Changing frequency to 5hz");
      freq = 5;
    }
  }
  ros::Publisher chatter_pub = n.advertise < std_msgs::String
      > ("chatter", 1000);

  // For service to changeBaseOutputString
  ros::ServiceServer server = n.advertiseService("change_string_output",
                                                 newtext);
  int count = 0;
  while (ros::ok()) {
    ROS_DEBUG_STREAM_ONCE("Current frequency: " << freq);

    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    std_msgs::String msg;

    std::stringstream ss;
    ss << text;
    msg.data = ss.str();

    ROS_INFO_STREAM(msg.data.c_str());

    /**
     * The publish() function represents the way of sending messages.
     */
    chatter_pub.publish(msg);

    // set translation
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(
        tf::Vector3(cos(ros::Time::now().toSec()),
                    sin(ros::Time::now().toSec()), 0.0));
    tf::Quaternion q;
    q.setRPY(0, 0, 1);

    // set rotation
    transform.setRotation(q);

    // broadcast the transform
    br.sendTransform(
        tf::StampedTransform(transform, ros::Time::now(), "world", "talk"));


    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }

  return 0;
}
