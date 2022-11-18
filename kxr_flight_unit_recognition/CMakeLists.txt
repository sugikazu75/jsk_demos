cmake_minimum_required(VERSION 3.0.2)
project(kxr_flight_unit_recognition)

find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  pcl_msgs
  tf
  rospy
  sensor_msgs
  )

catkin_package()
include_directories(
  ${catkin_INCLUDE_DIRS}
  )

add_executable(obstacles src/obstacles.cpp)
add_executable(example src/example.cpp)
target_link_libraries(obstacles ${catkin_LIBRARIES})
target_link_libraries(example ${catkin_LIBRARIES})
