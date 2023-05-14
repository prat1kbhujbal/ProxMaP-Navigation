#!/usr/bin/env python3
"""
Author: Pratik Bhujbal
License: MIT
"""

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage
from nav_msgs.msg import OccupancyGrid, Odometry
from cv_bridge import CvBridge


class CompressedImageToCostmap:
    """
    Class to convert compressed images to costmaps and publish them on a ROS topic
    """

    def __init__(self):
        """
        Constructor for CompressedImageToCostmap class
        """
        # Initialize the node
        rospy.init_node('compressed_image_to_costmap')

        # Create the publishers and subscribers
        self.costmap_pub = rospy.Publisher(
            '/costmap', OccupancyGrid, queue_size=10)
        self.image_sub = rospy.Subscriber(
            '/scan_to_pred/compressed',
            CompressedImage,
            self.image_callback)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)

        # Initialize CvBridge
        self.bridge = CvBridge()

        # Initialize odometry data
        self.odom = None

    def image_callback(self, compressed_image_msg):
        """
        Callback function to process incoming ROS messages and publish costmaps

        : param compressed_image_msg: CompressedImage ROS message
        """
        if self.odom is None:
            rospy.logwarn("Odometry data is not available yet.")
            return

        # Convert the compressed image message to an OpenCV image (grayscale)
        cv_image = self.bridge.compressed_imgmsg_to_cv2(
            compressed_image_msg, desired_encoding="mono8")
        cv_image = cv2.flip(cv_image, 0)

        # Convert the grayscale image to a costmap with origin from odometry
        costmap = self.image_to_costmap(cv_image)

        # Publish the costmap
        self.costmap_pub.publish(costmap)

    def odom_callback(self, odom_msg):
        """
        Callback function to update the odometry data

        : param odom_msg: Odometry ROS message
        """
        self.odom = odom_msg

    def image_to_costmap(self, cv_image):
        """
        Function to convert an OpenCV image to a costmap with origin from odometry

        : param cv_image: Input image in OpenCV format(grayscale)
        : return: OccupancyGrid message containing the costmap
        """
        # Convert the grayscale image to a numpy array
        image_array = np.array(cv_image, dtype=np.int8)

        # Create an OccupancyGrid message
        costmap = OccupancyGrid()
        costmap.header.stamp = rospy.Time.now()
        costmap.header.frame_id = "base_link"
        costmap.info.resolution = 0.0165  # Set the desired resolution of the costmap
        costmap.info.width = image_array.shape[1]
        costmap.info.height = image_array.shape[0]
        half_width = (costmap.info.width * costmap.info.resolution) / 2
        half_height = (costmap.info.height * costmap.info.resolution) / 2

        # Adjust the map's origin to align the bottom-middle part with the
        # robot's base
        costmap.info.origin.position.x = 0
        costmap.info.origin.position.y = - (
            costmap.info.width * costmap.info.resolution) / 2
        costmap.info.origin.orientation.w = 1.0

        flat_data = cv_image.flatten().tolist()

        # Scale the data to the range [0, 100]
        costmap.data = [int(i / 255.0 * 100) for i in flat_data]
        return costmap


if __name__ == '__main__':
    try:
        converter = CompressedImageToCostmap()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
