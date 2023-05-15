import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped
from math import sqrt


def map_callback(data):
    global costmap
    costmap = data


def goal_callback(data):
    global goal
    goal = data.pose.position


def odom_callback(data):
    global robot_position
    position = data.pose.pose.position
    robot_position = (position.x, position.y)


def get_average_occupancy(x, y, radius):
    global costmap

    if not costmap:
        rospy.loginfo("Waiting for map data")
        return None

    width = costmap.info.width
    height = costmap.info.height
    resolution = costmap.info.resolution
    origin_x = costmap.info.origin.position.x
    origin_y = costmap.info.origin.position.y

    map_x = int((x - origin_x) / resolution)
    map_y = int((y - origin_y) / resolution)

    occupancy_sum = 0
    count = 0
    for i in np.arange(-radius, radius + resolution, resolution):
        for j in np.arange(-radius, radius + resolution, resolution):
            if sqrt(i**2 + j**2) <= radius:
                x_index = map_x + int(i / resolution)
                y_index = map_y + int(j / resolution)

                if 0 <= x_index < width and 0 <= y_index < height:
                    index = y_index * width + x_index
                    occupancy = costmap.data[index]
                    if occupancy >= 0:
                        occupancy_sum += occupancy
                        count += 1

    if count > 0:
        return occupancy_sum / count
    else:
        return None


def adjust_speed_based_on_occupancy():
    global robot_position, goal

    robot_average_occupancy = get_average_occupancy(
        robot_position[0], robot_position[1], robot_radius)
    if goal is not None:
        goal_average_oc = get_average_occupancy(
            goal.x, goal.y, goal_radius)
    #     print("goal_average_occupancy: ", goal_average_oc)
    # # goal_average_occupancy = get_average_occupancy(goal.x, goal.y, goal_radius)
    #     print("robot_average_occupancy: ", robot_average_occupancy)

        if robot_average_occupancy is not None and goal_average_oc is not None:
            max_average_occupancy = min(
                robot_average_occupancy,
                goal_average_oc)

            if max_average_occupancy == -1:
                linear_speed = 0.1  # m/s
                angular_speed = 0.7
            else:

                linear_speed = 0.5  # m/s
                angular_speed = 1.0

            return linear_speed, angular_speed
        else:
            return None, None
    else:
        return None, None


if __name__ == '__main__':
    rospy.init_node('adaptive_speed_controller')

    costmap = None
    rospy.Subscriber("/map", OccupancyGrid, map_callback)
    goal = None
    rospy.Subscriber("/move_base_simple/goal", PoseStamped, goal_callback)

    # Set the robot's position, radius, and goal radius
    robot_position = (0.0, 0.0)
    rospy.Subscriber("/odom", Odometry, odom_callback)
    robot_radius = 0.5
    goal_radius = 0.5

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        linear_speed, angular_speed = adjust_speed_based_on_occupancy()

        if linear_speed is not None and angular_speed is not None:
            rospy.set_param(
                '/move_base/DWAPlannerROS/max_vel_x',
                linear_speed)
            rospy.set_param(
                '/move_base/DWAPlannerROS/max_vel_theta',
                angular_speed)
        rospy.loginfo(
            "Adjusted max_vel_x to: {} and max_rotational_vel to: {}".format(
                linear_speed, angular_speed))

        rate.sleep()
