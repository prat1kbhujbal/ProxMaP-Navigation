#!/usr/bin/env python

import rospy
import actionlib
import math
import os
from geometry_msgs.msg import Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import time
import csv
from tf.transformations import quaternion_from_euler
from nav_msgs.msg import Odometry
import tf

# Goals in the form of a dictionary with format: {goal_name: (x, y, theta)}
goals = {
    'goal_1': (4, 0, math.radians(45)),
    'goal_2': (6, 3, math.radians(90)),
    'goal_3': (6, 6, math.radians(135)),
    'goal_4': (3, 7, math.radians(180)),
    'goal_5': (1, 5, math.radians(-45)),
    'goal_6': (1, 2.5, math.radians(-45)),
    'goal_7': (0, 0, math.radians(0)),
}


def rotate_360():
    # Get the current robot pose
    current_pose = rospy.wait_for_message('/odom', Odometry)
    (_, _, current_yaw) = tf.transformations.euler_from_quaternion([
        current_pose.pose.pose.orientation.x,
        current_pose.pose.pose.orientation.y,
        current_pose.pose.pose.orientation.z,
        current_pose.pose.pose.orientation.w
    ])

    # Calculate the target yaw by adding 360 degrees (2 * pi radians) to the
    # current yaw
    target_yaw = current_yaw + 2 * math.pi

    goal = MoveBaseGoal()
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.header.frame_id = 'odom'

    # Set the position to the current position
    goal.target_pose.pose.position.x = current_pose.pose.pose.position.x
    goal.target_pose.pose.position.y = current_pose.pose.pose.position.y

    # Set the orientation to the target yaw
    quat = quaternion_from_euler(0, 0, target_yaw)
    goal.target_pose.pose.orientation = Quaternion(*quat)

    # Send the goal and wait for the result
    client.send_goal(goal)
    client.wait_for_result()

    return client.get_result()


def send_goal(goal_coords):
    rospy.sleep(2.0)
    goal = MoveBaseGoal()
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.header.frame_id = 'odom'
    goal.target_pose.pose.position.x = goal_coords[0]
    goal.target_pose.pose.position.y = goal_coords[1]

    # Convert theta to quaternion
    quat = quaternion_from_euler(0, 0, goal_coords[2])
    goal.target_pose.pose.orientation = Quaternion(*quat)

    # Send the goal and wait for the result
    client.send_goal(goal)
    client.wait_for_result()

    return client.get_result()


def generate_unique_filename(filename):
    index = 1
    base_filename, ext = os.path.splitext(filename)
    while os.path.exists(filename):
        filename = f"{base_filename}_{index}{ext}"
        index += 1
    return filename


rospy.init_node('move_base_goal_publisher')
client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
client.wait_for_server()

filename = 'time_to_reach_goals.csv'
unique_filename = generate_unique_filename(filename)

# Prepare the CSV file for writing
with open(unique_filename, 'w') as csvfile:
    fieldnames = ['goal_name', 'time_to_reach']
    csv_writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
    csv_writer.writeheader()

    total_time = 0  # Variable to accumulate the time taken for all goals

    # Iterate over the goals in the dictionary
    for goal_name, goal_coords in goals.items():
        # Record the start time
        start_time = time.time()

        # Send the goal
        rospy.loginfo(f'Sending {goal_name} at {goal_coords}')
        result = send_goal(goal_coords)
        # Calculate the time taken to reach the goal
        time_to_reach = time.time() - start_time
        rospy.loginfo(f'Time to reach {goal_name}: {time_to_reach} seconds')
        # rotate_360()

        # Write the data to the CSV file
        csv_writer.writerow(
            {'goal_name': goal_name, 'time_to_reach': time_to_reach})

        # Accumulate the time
        total_time += time_to_reach

    # Write the total time to the CSV file
    csv_writer.writerow(
        {'goal_name': 'Total Time', 'time_to_reach': total_time})

    rospy.loginfo(f'Total time to reach all goals: {total_time} seconds')
