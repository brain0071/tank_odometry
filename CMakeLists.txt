cmake_minimum_required(VERSION 2.8.3)
project(tank_odometry)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  rospy
  std_msgs
  message_generation
)

add_message_files(FILES 
  AprilTagDetection.msg
  AprilTagDetectionArray.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  nav_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS 
  rospy 
  std_msgs 
  message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

