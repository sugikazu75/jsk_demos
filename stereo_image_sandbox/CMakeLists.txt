cmake_minimum_required(VERSION 2.8.3)
project(stereo_image_sandbox)

find_package(catkin REQUIRED COMPONENTS
  camera_info_manager
  cv_bridge
  jsk_topic_tools
  nodelet
  roscpp
  sensor_msgs
)

find_package(OpenCV REQUIRED)
message("OPENCV ${OpenCV_VERSION} FOUND")

catkin_package(
  CATKIN_DEPENDS
  jsk_topic_tools
  camera_info_manager
  sensor_msgs
  DEPENDS
  INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIRS}
)

# Build nodelet executables
include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

macro(jsk_add_nodelet _nodelet_cpp _nodelet_class _single_nodelet_exec_name)
  if(NOT EXISTS "${PROJECT_SOURCE_DIR}/test/${_single_nodelet_exec_name}.test")
    list(APPEND ${PROJECT_NAME}_nodes_without_test ${_nodelet_cpp})
  endif()
  jsk_nodelet(${_nodelet_cpp} ${_nodelet_class} ${_single_nodelet_exec_name}
    ${PROJECT_NAME}_nodelet_sources ${PROJECT_NAME}_nodelet_executables)
endmacro()

# add_executable(dummy src/split_image.cpp)
jsk_add_nodelet(src/split_image.cpp "stereo_image_sandbox/SplitImage" "split_image")
