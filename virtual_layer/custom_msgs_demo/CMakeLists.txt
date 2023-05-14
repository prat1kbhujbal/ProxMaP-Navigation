# ##############################################################################
# Version setup
# ##############################################################################

set(PACKAGE_NAME custom_msgs)
file(STRINGS ${CMAKE_CURRENT_LIST_DIR}/VERSION PROJECT_VERSION)
message(STATUS "Project version: ${PROJECT_VERSION}")

# ##############################################################################
# Project information
# ##############################################################################

cmake_minimum_required(VERSION 3.5.1)
project(${PACKAGE_NAME} LANGUAGES C CXX VERSION ${PROJECT_VERSION})
set(PROJECT_RELEASE_TYPE release) # alpha, beta, release

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake_modules")
include(declarations)

# ##############################################################################
# Load and setup packages
# ##############################################################################

find_package(catkin REQUIRED
             COMPONENTS roscpp
                        rospy
                        std_msgs
                        message_generation
                        geometry_msgs
                        nav_msgs)

# ##############################################################################
# Generate messages and configurations
# ##############################################################################

add_message_files(DIRECTORY
                  msg/virtual_layer
                  FILES
                  Form.msg
                  Obstacles.msg
                  Zone.msg)

generate_messages(DEPENDENCIES
                  std_msgs
                  geometry_msgs
                  nav_msgs)

# ##############################################################################
# Setup catkin packages
# ##############################################################################

catkin_package(CATKIN_DEPENDS
               roscpp
               rospy
               message_runtime
               std_msgs
               geometry_msgs
               nav_msgs)
