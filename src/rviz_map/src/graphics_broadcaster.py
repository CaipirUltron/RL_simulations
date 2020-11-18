#!/usr/bin/env python

import rospy
import numpy as np
import tf
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, PointStamped
from nav_msgs.msg import GridCells

def placeGridCells(data):

    rospy.loginfo("Received cell matrix.")

def place_reference(data):

    # data.point.x
    # data.point.y

    rospy.loginfo("Point clicked.: (%s, %s)", data.point.x, data.point.y)

if __name__ == '__main__':
    try:
        gridMap = GridCells()
        gridMap.header.frame_id = "grid_frame"
        gridMap.cell_height = 1.0
        gridMap.cell_width = 1.0

        world_tf = tf.TransformBroadcaster()

        # Subscribers
        click_sub = rospy.Subscriber("clicked_point", PointStamped, place_reference)
        occupancy_sub = rospy.Subscriber("occupancyMatrix", PointStamped, placeGridCells)

        # Publishers
        cells_publisher = rospy.Publisher('gridMap', GridCells, queue_size=1)

        # Initialize node
        rospy.init_node('gridMap', anonymous=True)
        rospy.loginfo("Starting grid map!...")

        rate = rospy.Rate(1) # 10hz
        # Publish map grid
        while not rospy.is_shutdown():
            
            cells_publisher.publish(gridMap)

            random_x = np.random.randint(-5, high=5) + 0.5
            random_y = np.random.randint(-5, high=5) + 0.5
            point = Point(random_x, random_y, 0)
            gridMap.cells.append(point)
            
            world_tf.sendTransform((0,0,0),(0,0,0,1),rospy.Time.now(),"grid_frame", "world")
            
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
