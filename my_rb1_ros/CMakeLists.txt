cmake_minimum_required(VERSION 3.0.2)
project(my_rb1_ros)
## Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  nav_msgs
)


add_service_files(
  FILES
  Rotate.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES my_rb1_ros
  CATKIN_DEPENDS roscpp rospy std_msgs tf nav_msgs
  DEPENDS system_lib
)



add_executable(rotate_service src/rotate_service.cpp)

  target_link_libraries(rotate_service
   ${catkin_LIBRARIES}
 )

 