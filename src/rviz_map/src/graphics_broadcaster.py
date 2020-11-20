#!/usr/bin/env python

import rospy
import numpy as np
import tf

from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Pose
from nav_msgs.msg import OccupancyGrid

# from map import Map
# from robot import Robot

# This function receives an numpy occupancy map and publishes it in rviz_map
def update_map():

    map_size = 10

    # Send transformation
    world_tf.sendTransform((0,0,0),(0,0,0,1),rospy.Time.now(),"map_frame", "world")

    # Create rviz map
    rviz_map = OccupancyGrid()
    rviz_map.header.frame_id = "map_frame"

    rviz_map.header.seq = rviz_map.header.seq + 1
    rviz_map.header.stamp = rospy.Time.now()
    rviz_map.info.map_load_time = rospy.Time.now()
    rviz_map.info.resolution = 1.0
    rviz_map.info.width = map_size
    rviz_map.info.height = map_size
    rviz_map.info.origin.position.x = -map_size/2
    rviz_map.info.origin.position.y = -map_size/2

    map_array = np.random.randint(0, 2, size = (map_size, map_size))
    rviz_map.data = 100 * map_array.flatten()

    map_publisher.publish(rviz_map)


# This function receives a Point ROS msg and publishes the corresponding robot position on Rviz
def update_robot_state(data):

    world_tf.sendTransform((0,0,0),(0,0,0,1),rospy.Time.now(),"robot_frame", "world")

    robot_position_marker.header.frame_id = "robot_frame"
    robot_position_marker.type = robot_position_marker.CUBE
    robot_position_marker.action = robot_position_marker.ADD
    robot_position_marker.scale.x = 1.0
    robot_position_marker.scale.y = 1.0
    robot_position_marker.scale.z = 0.0
    robot_position_marker.color.a = 1.0
    robot_position_marker.color.r = 0.0
    robot_position_marker.color.g = 0.0
    robot_position_marker.color.b = 0.0
    robot_position_marker.pose.position.x = data.position.x
    robot_position_marker.pose.position.y = data.position.y
    robot_position_marker.pose.position.z = 0
    robot_position_marker.pose.orientation.x = 0.0
    robot_position_marker.pose.orientation.y = 0.0
    robot_position_marker.pose.orientation.z = 0.0
    robot_position_marker.pose.orientation.w = 1.0

    

    robot_marker_publisher.publish(robot_position_marker)
    angle_marker_publisher.publish(robot_angle_arrow)

def update_reference(data):

    world_tf.sendTransform((0,0,0),(0,0,0,1),rospy.Time.now(),"goal_frame", "world")

    goal_marker.header.frame_id = "goal_frame"
    goal_marker.type = goal_marker.CUBE
    goal_marker.action = goal_marker.ADD
    goal_marker.scale.x = 1.0
    goal_marker.scale.y = 1.0
    goal_marker.scale.z = 0.0
    goal_marker.color.a = 1.0
    goal_marker.color.r = 0.0
    goal_marker.color.g = 0.0
    goal_marker.color.b = 0.0
    goal_marker.pose.position.x = data.point.x
    goal_marker.pose.position.y = data.point.y
    goal_marker.pose.position.z = 0
    goal_marker.pose.orientation.x = 0.0
    goal_marker.pose.orientation.y = 0.0
    goal_marker.pose.orientation.z = 0.0
    goal_marker.pose.orientation.w = 1.0

    goal_marker_publisher.publish(goal_marker)

if __name__ == '__main__':
    try:
        
        # Setup tf tree and rviz objects
        world_tf = tf.TransformBroadcaster()
        rviz_map = OccupancyGrid()
        robot_position_marker = Marker()
        goal_marker = Marker()

        # Subscribers
        # click_sub = rospy.Subscriber("clicked_point", PointStamped, update_reference)
        robot_sub = rospy.Subscriber("robot_state", Pose, update_robot_state)

        # Publishers
        map_publisher = rospy.Publisher('map', OccupancyGrid, queue_size=1)
        robot_marker_publisher = rospy.Publisher('robot_marker', Marker, queue_size=1)
        angle_marker_publisher = rospy.Publisher('angle_marker', Marker, queue_size=1)
        goal_marker_publisher = rospy.Publisher('goal_marker', Marker, queue_size=1)

        # Initialize node
        rospy.init_node('graphics_broadcaster', anonymous=True)
        rospy.loginfo("Starting grid map!...")
        
        rate = rospy.Rate(10)  # 10hz
        while not rospy.is_shutdown():
            update_map()
            rate.sleep()
        # rospy.spin()

    except rospy.ROSInterruptException:
        pass
