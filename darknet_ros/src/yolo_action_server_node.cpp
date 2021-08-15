/*
 * yolo_action_server_node.cpp
 *
 * Created on: August 15, 2021
 *      Author: Marco Gabriele Fedozzi
 *   Institute: University of Genoa, MSc Robotics Engineering
 */

#include <ros/ros.h>
#include <darknet_ros/YoloActionServer.hpp>

int main(int argc, char** argv) {
  ros::init(argc, argv, "darknet_ros_as");
  ros::NodeHandle nodeHandle("~");
  darknet_ros::YoloActionServer yoloActionServer(nodeHandle);

  ros::spin();
  return 0;
}
