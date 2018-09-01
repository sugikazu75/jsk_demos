cmake_minimum_required(VERSION 2.8.3)
project(jsk_pr2_plugs)

find_package(catkin REQUIRED COMPONENTS
  jsk_pr2_startup
  jsk_recognition_msgs
  jsk_topic_tools
  message_filter
  nodelet
  pcl_msgs
  pr2eus
  roscpp
  rospy
  tf
  tf2_ros
)

catkin_package(
  LIBRARIES ${PROJECT_NAME}
)
