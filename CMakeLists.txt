cmake_minimum_required(VERSION 2.8.3)
project(state_interpreter)

add_compile_options(-std=c++11)

find_package(autoware_build_flags REQUIRED)

find_package(autoware_msgs REQUIRED)
find_package(autoware_config_msgs REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  sound_play
  std_msgs
  geometry_msgs
  diagnostic_msgs
  autoware_msgs
  autoware_config_msgs
  message_generation
)

add_message_files(
  FILES
  Int32Stamped.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  roscpp
  sound_play
  std_msgs
  geometry_msgs
  diagnostic_msgs
  autoware_msgs
  autoware_config_msgs
  message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${autoware_msgs_INCLUDE_DIRS}
  ${autoware_config_msgs_INCLUDE_DIRS}
)

add_executable(state_interpreter_node
src/state_interpreter_node.cpp
src/state_interpreter.cpp)

target_link_libraries(state_interpreter_node
  ${catkin_LIBRARIES}
  -lyaml-cpp
)

add_dependencies(state_interpreter_node ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)

install(TARGETS state_interpreter_node
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY sample_dataset/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/sample_dataset
        PATTERN ".svn" EXCLUDE)

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        PATTERN ".svn" EXCLUDE)

