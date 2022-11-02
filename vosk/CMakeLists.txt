cmake_minimum_required(VERSION 2.8.3)
project(vosk)

find_package(catkin REQUIRED COMPONENTS
  catkin_virtualenv
)

catkin_package(
  CATKIN_DEPENDS
)

catkin_generate_virtualenv(
  PYTHON_INTERPRETER python3
)


file(GLOB SCRIPTS_FILES node_scripts/*.py)
catkin_install_python(
  PROGRAMS ${SCRIPTS_FILES}
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(FILES requirements.txt
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# download and install trained data
add_custom_target(${PROJECT_NAME}_install_trained_data ALL COMMAND ${PROJECT_SOURCE_DIR}/scripts/install_trained_data.py)
# add_custom_target(${PROJECT_NAME}_install_sample_data ALL COMMAND ${PROJECT_SOURCE_DIR}/scripts/install_sample_data.py)
