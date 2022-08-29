cmake_minimum_required(VERSION 2.8.3)
project(jsk_teaching_object)

find_package(catkin REQUIRED COMPONENTS
  jsk_data)

# ------------------------------------------------------------------------------------
# Download
# ------------------------------------------------------------------------------------

# download and install sample data
add_custom_target(${PROJECT_NAME}_install_sample_data ALL COMMAND python$ENV{ROS_PYTHON_VERSION} ${PROJECT_SOURCE_DIR}/scripts/install_sample_data.py)

# download and install trained data
add_custom_target(${PROJECT_NAME}_install_trained_data ALL COMMAND python$ENV{ROS_PYTHON_VERSION} ${PROJECT_SOURCE_DIR}/scripts/install_trained_data.py)

catkin_package()
