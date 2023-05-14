#!/usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Twist
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry


def costmap_callback(costmap):
    # Store the costmap message for later use
    costmap_callback.costmap = costmap


def update_robot_speed(event):
    # Get the robot's current position in the costmap
    robot_x = costmap_callback.costmap.info.origin.position.x + \
        update_robot_speed.robot_pose.pose.pose.position.x
    robot_y = costmap_callback.costmap.info.origin.position.y + \
        update_robot_speed.robot_pose.pose.pose.position.y

    # Set the radius of the circular region around the robot to consider for
    # cost
    radius = 0.5  # meters

    # Calculate the row and column indices of the cells in the circular region
    info = costmap_callback.costmap.info
    map_width = info.width
    map_resolution = info.resolution
    robot_x_index = int((robot_x - info.origin.position.x) / map_resolution)
    robot_y_index = int((robot_y - info.origin.position.y) / map_resolution)
    min_x_index = max(robot_x_index - int(radius / map_resolution), 0)
    max_x_index = min(
        robot_x_index + int(radius / map_resolution),
        map_width - 1)
    min_y_index = max(robot_y_index - int(radius / map_resolution), 0)
    max_y_index = min(
        robot_y_index + int(radius / map_resolution),
        len(costmap_callback.costmap.data) // map_width - 1)

    # Calculate the average cost of the cells in the circular region
    cost_sum = 0
    count = 0
    for y in range(min_y_index, max_y_index + 1):
        for x in range(min_x_index, max_x_index + 1):
            cell_index = y * map_width + x
            cost_sum += costmap_callback.costmap.data[cell_index]
            count += 1
    avg_cost = cost_sum / count

    # Set the robot's speed based on the average cost in the circular region
    # if avg_cost > 90:
    #     # rospy.set_param('/move_base/TebLocalPlannerROS/max_vel_x', 0.5)
    # elif avg_cost > 80:
    #     # rospy.set_param('/move_base/TebLocalPlannerROS/max_vel_x', 0.75)
    # else:
    #     # rospy.set_param('/move_base/TebLocalPlannerROS/max_vel_x', 1.0)
    # if avg_cost > 90:
    #     print("avg_cost > 90", avg_cost)
    # elif avg_cost > 80:
    #     print("avg_cost > 80", avg_cost)
    # else:
    #     print("avg_cost ", avg_cost)
    print("avg_cost ", avg_cost)

if __name__ == '__main__':
    try:
        # Subscribe to the robot's pose topic
        rospy.init_node('costmap_subscriber')
        robot_pose_sub = rospy.Subscriber(
            'odom',
            Odometry,
            lambda msg: setattr(update_robot_speed, 'robot_pose', msg))

        # Subscribe to the global costmap topic
        costmap_sub = rospy.Subscriber(
            'move_base/local_costmap/costmap',
            OccupancyGrid,
            costmap_callback)

        # Call the update_robot_speed function at a rate of 1 Hz
        rospy.Timer(rospy.Duration(1.0), update_robot_speed)

        rospy.spin()
    except rospy.ROSInterruptException:
        pass
