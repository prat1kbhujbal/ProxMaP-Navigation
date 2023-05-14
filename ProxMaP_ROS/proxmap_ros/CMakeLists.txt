cmake_minimum_required(VERSION 3.0.2)
project(proxmap_ros)


add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2_ros
  tf2_geometry_msgs
  vision_opencv
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES proxmap_ros
  CATKIN_DEPENDS rospy sensor_msgs geometry_msgs nav_msgs tf2_ros tf2_geometry_msgs vision_opencv
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

# Installation
catkin_install_python(PROGRAMS
  scripts/1.py
  scripts/2.py
  scripts/4.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Testing
# if (CATKIN_ENABLE_TESTING)
#   find_package(rostest REQUIRED)
#   add_rostest(test/test_proxmap.test)
# endif()
