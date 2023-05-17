#!/usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid, Odometry
from map_msgs.msg import OccupancyGridUpdate
from dynamic_reconfigure.client import Client
import numpy as np

# Robot radius in meters
ROBOT_RADIUS = 0.5

# DWA parameter names
MAX_VEL_TRANS = 'max_vel_trans'
MAX_VEL_X = 'max_vel_x'
MAX_ROT_VEL = 'max_rot_vel'
MIN_ROT_VEL = 'min_rot_vel'

# Global variable to store the robot's position
robot_position = None
global_costmap = None
global_costmap_info = None


def set_dwa_params(client, max_vel_x):
    params = {
        MAX_VEL_X: max_vel_x,
        MAX_VEL_TRANS: max_vel_x
    }
    client.update_configuration(params)


def odom_callback(msg):
    global robot_position
    robot_position = msg.pose.pose.position


def global_costmap_callback(msg):
    global global_costmap, global_costmap_info

    width = msg.info.width
    height = msg.info.height
    resolution = msg.info.resolution

    # Convert costmap data to a NumPy array
    global_costmap = np.array(msg.data).reshape(height, width)
    global_costmap_info = msg.info


def costmap_callback(msg):
    global robot_position, global_costmap, global_costmap_info

    if robot_position is None:
        return

    if global_costmap is None:
        rospy.logerr(
            "Global costmap is not initialized. Cannot apply updates.")
        return

    # Apply the update to the global costmap
    update_data = np.array(msg.data).reshape(msg.height, msg.width)
    global_costmap[msg.y:msg.y + msg.height,
                   msg.x:msg.x + msg.width] = update_data

    try:
        # Get costmap metadata
        width, height = global_costmap.shape
        resolution = global_costmap_info.resolution
        origin = global_costmap_info.origin

        # Calculate the robot's position in the costmap
        robot_x = int((robot_position.x - origin.position.x) / resolution)
        robot_y = int((robot_position.y - origin.position.y) / resolution)

        # Calculate the number of cells in the robot's radius
        cells_in_radius = int(ROBOT_RADIUS / resolution)

        # Extract the submap around the robot's position
        submap = global_costmap[robot_y -
                                cells_in_radius:robot_y +
                                cells_in_radius, robot_x -
                                cells_in_radius:robot_x +
                                cells_in_radius]

        # Calculate the average cost in the submap
        avg_cost = np.mean(submap)
        # print(f"Average cost: {avg_cost}")

        # Set DWA parameters based on the average cost
        max_vel_x = 0.4 if avg_cost >= 0 else 0.2
        set_dwa_params(dwa_client, max_vel_x)

    except Exception as e:
        rospy.logerr(f"Error in costmap_callback: {e}")


if __name__ == '__main__':
    try:
        rospy.init_node('adaptive_speed')

        # Subscribe to the /odom and /map topics
        rospy.Subscriber('/odom', Odometry, odom_callback)
        rospy.Subscriber(
            '/map',
            OccupancyGrid,
            global_costmap_callback)
        rospy.Subscriber(
            '/move_base/global_costmap/costmap_updates',
            OccupancyGridUpdate,
            costmap_callback)

        # Initialize the dynamic reconfigure client for DWA parameters
        dwa_client = Client('/move_base/DWAPlannerROS')

        rospy.spin()

    except rospy.ROSInterruptException as e:
        rospy.logerr(f"Error in main: {e}")
