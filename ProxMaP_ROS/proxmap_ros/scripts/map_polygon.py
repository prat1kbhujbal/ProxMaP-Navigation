#!/usr/bin/env python
"""
Extracts a polygon from a given image and publishes it on a ROS topic wrt robot.

Author: Pratik Bhujbal
License: MIT
"""

import cv2
import rospy
import tf2_ros
import geometry_msgs.msg
import tf2_geometry_msgs
import numpy as np
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from geometry_msgs.msg import PolygonStamped
from geometry_msgs.msg import Polygon, Point32


class PolygonExtractor:
    """
    Class to extract polygons from images and publish them on a ROS topic
    """

    def __init__(self):
        """
        Constructor for PolygonExtractor class
        """
        rospy.init_node('polygon_extractor')
        rospy.Subscriber(
            '/scan_to_pred/compressed', CompressedImage, self.callback)
        self.pub = rospy.Publisher(
            '/convex_hull_polygon', PolygonStamped, queue_size=1)

        # Add a tf buffer and listener
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)

    def callback(self, msg):
        """
        Callback function to process incoming ROS messages and publish transformed polygons

        :param msg: CompressedImage ROS message
        """
        # Convert the compressed image to a cv2 image
        bridge = CvBridge()
        cv_image = bridge.compressed_imgmsg_to_cv2(
            msg, desired_encoding='bgr8')
        cv_image = cv2.flip(cv_image, 0)

        # Extract the polygon from the cv2 image
        polygon = self.extract_polygon(cv_image)

        # Get the transform between the robot's base_link and the map frame
        try:
            transform = self.tfBuffer.lookup_transform(
                'map', 'base_link', rospy.Time(0), rospy.Duration(1.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr("Failed to get transform: {}".format(e))
            return

        # Apply the transform to the polygon's points
        transformed_polygon = Polygon()
        for point in polygon.points:
            point_stamped = geometry_msgs.msg.PointStamped()
            point_stamped.header.frame_id = "base_link"
            point_stamped.header.stamp = rospy.Time.now()
            point_stamped.point = point
            transformed_point_stamped = tf2_geometry_msgs.do_transform_point(
                point_stamped,
                transform)
            transformed_polygon.points.append(transformed_point_stamped.point)

        # Publish the transformed polygon on a new ROS topic
        polygon_msg = PolygonStamped()
        polygon_msg.header.stamp = rospy.Time.now()
        polygon_msg.header.frame_id = 'map'
        polygon_msg.polygon = transformed_polygon
        self.pub.publish(polygon_msg)

    def get_bottom_most_points(self, contour):
        """
        Helper function to get the bottom two points of a contour

        :param contour: OpenCV contour
        :return: List of two bottommost points
        """
        contour_reshaped = contour.reshape((-1, 2))
        sorted_points = sorted(
            contour_reshaped,
            key=lambda point: point[1],
            reverse=True)
        bottom_most_points = sorted_points[:2]
        return bottom_most_points

    def extract_polygon(self, image):
        """
        Function to extract polygon from an input image

        :param image: Input image in OpenCV format
        :return: Polygon in geometry_msgs/Polygon format
        """
        # Increase the resolution of the input image (if needed)
        image = cv2.resize(
            image, (image.shape[1] * 2, image.shape[0] * 2),
            interpolation=cv2.INTER_AREA)
        # Convert the image to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Apply Gaussian blur to reduce noise
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)

        # Apply Canny edge detection
        edges = cv2.Canny(blurred, 50, 150)

        # Dilate the edges to make them more pronounced
        kernel = np.ones((10, 10), np.uint8)
        # Use  to adjust the updated polygon
        # dilated = cv2.dilate(edges, kernel, iterations=1)

        # Perform morphological closing to join nearby contours
        closed = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel, iterations=2)

        # Find contours in the closed image
        contours, _ = cv2.findContours(
            closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Find the contour with the largest area
        max_area = 0
        max_contour = None
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > max_area:
                max_area = area
                max_contour = contour

        # Approximate the contour to a polygon with a smaller epsilon value
        epsilon = 0.00005 * cv2.arcLength(max_contour, True)
        approx = cv2.approxPolyDP(max_contour, epsilon, True)

        # Convert the approximated contour points to a geometry_msgs/Polygon
        polygon = Polygon()
        for point in approx:
            pt = Point32()
            # Adjust the scaling factor according to the new resolution
            pt.x = point[0][0] * 0.00825
            pt.y = point[0][1] * 0.00825 - 2.1
            pt.z = 0  # Assuming the image is 2D, set z to 0
            polygon.points.append(pt)

        return polygon


if __name__ == '__main__':
    rospy.init_node('polygon_extractor')
    polygon_extractor = PolygonExtractor()
    rospy.spin()
