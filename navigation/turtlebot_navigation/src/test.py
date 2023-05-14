#!/usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid


def costmap_callback(costmap):
    # Store the costmap message for later use
    costmap_callback.costmap = costmap


if __name__ == '__main__':
    try:
        # Subscribe to the global costmap topic
        rospy.init_node('costmap_subscriber')
        costmap_sub = rospy.Subscriber(
            'move_base/global_costmap/costmap',
            OccupancyGrid,
            costmap_callback)

        # Wait for the first costmap message to arrive
        while not hasattr(costmap_callback, 'costmap'):
            pass

        # Get the costmap information
        info = costmap_callback.costmap.info
        map_width = info.width
        map_resolution = info.resolution

        # Get the cost of the cell at (x, y) in the costmap
        x = 10  # meters
        y = 5   # meters
        x_index = int((x - info.origin.position.x) / map_resolution)
        y_index = int((y - info.origin.position.y) / map_resolution)
        cell_index = y_index * map_width + x_index
        cost = costmap_callback.costmap.data[cell_index]

        rospy.loginfo('Cost of cell at (%.2f, %.2f) in the costmap: %d', x, y, cost)

        rospy.spin()
    except rospy.ROSInterruptException:
        pass
