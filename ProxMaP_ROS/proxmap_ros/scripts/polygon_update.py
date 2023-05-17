#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PolygonStamped, Point
from custom_msgs.msg import Obstacles, Form
from std_srvs.srv import Empty


class PolygonToObstacles:
    def __init__(self):
        rospy.init_node('polygon_to_obstacles')

        self.obstacles_pub = rospy.Publisher(
            '/obstacles', Obstacles, queue_size=1)
        rospy.Subscriber(
            '/convex_hull_polygon',
            PolygonStamped,
            self.polygon_callback, queue_size=1)

    def polygon_callback(self, polygon_stamped):
        rate = rospy.Rate(1)
        obstacles = Obstacles()
        form = Form()

        for point32 in polygon_stamped.polygon.points:
            point = Point(x=point32.x, y=point32.y, z=point32.z)
            form.form.append(point)

        obstacles.list.append(form)
        self.obstacles_pub.publish(obstacles)
        rate.sleep()


if __name__ == '__main__':

    polygon_to_obstacles = PolygonToObstacles()
    rospy.spin()
