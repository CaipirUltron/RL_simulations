#!/usr/bin/env python

import rospy
import numpy as np
import tf
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, PointStamped
from nav_msgs.msg import GridCells

def place_reference(data):

    # data.point.x
    # data.point.y

    rospy.loginfo("Point clicked.: (%s, %s)", data.point.x, data.point.y)

if __name__ == '__main__':
    try:
        gridMap = GridCells()
        gridMap.header.frame_id = "grid_frame"
        gridMap.cell_height = 0.3
        gridMap.cell_width = 0.3

        firstPoint = Point(1.,1.,0.)
        gridMap.cells.append(firstPoint)

        world_tf = tf.TransformBroadcaster()

        # Subscribers
        click_sub = rospy.Subscriber("clicked_point", PointStamped, place_reference)

        # Publishers
        cells_publisher = rospy.Publisher('gridMap', GridCells, queue_size=10)

        # Initialize node
        rospy.init_node('gridMap', anonymous=True)
        rospy.loginfo("Starting grid map!...")

        rate = rospy.Rate(50) # 10hz
        # Publish map grid
        while not rospy.is_shutdown():
            cells_publisher.publish(gridMap)
            world_tf.sendTransform((0,0,0),(0,0,0,1),rospy.Time.now(),"grid_frame", "world")
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
