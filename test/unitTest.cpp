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
 *@file      test.cpp
 *@author    Pruthvikumar Sanghavi
 *@copyright MIT
 *@brief Testing talker node
 */

#include <gtest/gtest.h>
#include <ros/ros.h>
#include <ros/service_client.h>
#include "beginner_tutorials/change_string_output.h"
#include "std_msgs/String.h"

/**
 * @brief      Testing the service existence
 * @param      testTalkerNode         gtest framework
 * @param      serviceExsistanceTest  Name of the test
 */
TEST(testTalkerNode, testServiceExsistance) {
  // Connects the node with ROS
  ros::NodeHandle n;

  // client to server registration
  auto client = n.serviceClient<beginner_tutorials::change_string_output>
  ("change_string_output");
  // A wait message
  EXPECT_TRUE(client.waitForExistence(ros::Duration(5)));
}

/**
 * @brief      Tests if change_string_output service can replace the text 
 * @param      testTalkerNode            gtest framework
 * @param      testServiceMessageUpdate  Name of the test
 */
TEST(testTalkerNode, testServiceMessageUpdate) {
  // node handle creation to initialize the connection between nodes
  ros::NodeHandle n;

  // client to server registration
  auto client = n.serviceClient<beginner_tutorials::change_string_output>
  ("change_string_output");
  // Initialize the service to srv object
  beginner_tutorials::change_string_output srv;

  // changing the original text to some new text
  srv.request.originaltext = "input";

  // request
  client.call(srv.request, srv.response);

  // test case that shows whether input and output are equal
  EXPECT_STREQ("input", srv.response.newtext.c_str());
}

