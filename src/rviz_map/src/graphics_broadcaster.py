#!/usr/bin/env python

import rospy
import numpy as np
import tf

from rospy.numpy_msg import numpy_msg
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Pose, PoseStamped
from nav_msgs.msg import OccupancyGrid
from tf.transformations import quaternion_from_euler

from map import Map
from robot import Robot

class GridWorldSimulation():

    def __init__(self, robot):

        # Get map size
        self.robot = robot
        self.map_size = robot.world.size

        # ROS Subscribers
        # click_sub = rospy.Subscriber("clicked_point", PointStamped, update_reference)
        self.robot_sub = rospy.Subscriber("robot_state", Pose, self.update_robot)

        # ROS Publishers
        self.map_publisher = rospy.Publisher('map', OccupancyGrid, queue_size=1)
        self.robot_marker_publisher = rospy.Publisher('robot_marker', Marker, queue_size=1)
        self.angle_marker_publisher = rospy.Publisher('angle_marker', PoseStamped, queue_size=1)
        self.goal_marker_publisher = rospy.Publisher('goal_marker', Marker, queue_size=1)

        # Setup tf tree and rviz objects
        self.world_tf = tf.TransformBroadcaster()
        self.rviz_map = OccupancyGrid()
        self.robot_position_marker = Marker()
        self.robot_angle_marker = PoseStamped()
        self.goal_marker = Marker()

        # Initialize rviz map
        self.rviz_map.header.frame_id = "map_frame"
        self.rviz_map.header.seq = self.rviz_map.header.seq + 1
        self.rviz_map.header.stamp = rospy.Time.now()
        self.rviz_map.info.map_load_time = rospy.Time.now()
        self.rviz_map.info.resolution = 1.0
        self.rviz_map.info.width = self.map_size
        self.rviz_map.info.height = self.map_size
        self.rviz_map.info.origin.position.x = -self.map_size/2
        self.rviz_map.info.origin.position.y = -self.map_size/2

        # Initialize robot position marker
        self.robot_position_marker.header.frame_id = "robot_frame"
        self.robot_position_marker.type = self.robot_position_marker.CYLINDER
        self.robot_position_marker.action = self.robot_position_marker.ADD
        self.robot_position_marker.scale.x = 1.0
        self.robot_position_marker.scale.y = 1.0
        self.robot_position_marker.scale.z = 0.1
        self.robot_position_marker.color.a = 1.0
        self.robot_position_marker.color.r = 0.0
        self.robot_position_marker.color.g = 0.0
        self.robot_position_marker.color.b = 1.0
        self.robot_position_marker.pose.position.x = 0.0
        self.robot_position_marker.pose.position.y = 0.0
        self.robot_position_marker.pose.position.z = -0.1
        self.robot_position_marker.pose.orientation.x = 0.0
        self.robot_position_marker.pose.orientation.y = 0.0
        self.robot_position_marker.pose.orientation.z = 0.0
        self.robot_position_marker.pose.orientation.w = 1.0

        # Initialize robot orientation marker
        self.robot_angle_marker.header.frame_id = "robot_frame"
        self.robot_angle_marker.pose.position.x = 0.0
        self.robot_angle_marker.pose.position.y = 0.0
        self.robot_angle_marker.pose.position.z = 0.0
        self.robot_position_marker.pose.orientation.x = 0.0
        self.robot_position_marker.pose.orientation.y = 0.0
        self.robot_position_marker.pose.orientation.z = 0.0
        self.robot_position_marker.pose.orientation.w = 1.0

        # Initialize goal position marker
        self.goal_marker.header.frame_id = "goal_frame"
        self.goal_marker.type = self.goal_marker.CUBE
        self.goal_marker.action = self.goal_marker.ADD
        self.goal_marker.scale.x = 1.0
        self.goal_marker.scale.y = 1.0
        self.goal_marker.scale.z = 0.0
        self.goal_marker.color.a = 1.0
        self.goal_marker.color.r = 0.0
        self.goal_marker.color.g = 0.0
        self.goal_marker.color.b = 0.0
        self.goal_marker.pose.position.x = 0.0
        self.goal_marker.pose.position.y = 0.0
        self.goal_marker.pose.position.z = 0.0
        self.goal_marker.pose.orientation.x = 0.0
        self.goal_marker.pose.orientation.y = 0.0
        self.goal_marker.pose.orientation.z = 0.0
        self.goal_marker.pose.orientation.w = 1.0

        # Initial map update
        self.update_map()

    # This function receives an numpy occupancy map and publishes it in rviz_map
    def update_map(self):

        self.rviz_map.header.seq = self.rviz_map.header.seq + 1
        self.rviz_map.header.stamp = rospy.Time.now()
        self.rviz_map.info.map_load_time = rospy.Time.now()
        self.rviz_map.data = 100 * self.robot.world.map.flatten()

        # Send map transformation
        self.world_tf.sendTransform((0,0,0),(0,0,0,1),rospy.Time.now(),"map_frame", "world")

        self.map_publisher.publish(self.rviz_map)
        self.goal_marker_publisher.publish(self.goal_marker)


    # This function receives a Pose ROS msg and updates the corresponding transformation
    def update_robot(self):

        robot_marker_position_x = -self.map_size/2 + self.robot.position[0] + 0.5
        robot_marker_position_y = self.map_size/2 - self.robot.position[1] - 0.5
        robot_quat = quaternion_from_euler(0, 0, np.radians(self.robot.angle))

        self.world_tf.sendTransform((robot_marker_position_x,robot_marker_position_y,0),robot_quat,rospy.Time.now(),"robot_frame", "world")

        # Publishes Rviz graphical objects
        self.robot_marker_publisher.publish(self.robot_position_marker)
        self.angle_marker_publisher.publish(self.robot_angle_marker)


    # This function receives a Point ROS msg and updates the corresponding transformation
    def update_goal(self, goal_position):

        robot_marker_position_x = -self.map_size/2 + goal_position[0] + 0.5
        robot_marker_position_y = self.map_size/2 - goal_position[1] - 0.5
        self.world_tf.sendTransform((robot_marker_position_x,robot_marker_position_y,0),(0,0,0,1),rospy.Time.now(),"goal_frame", "world")


if __name__ == '__main__':
    try:
        
        # Define world map and robot
        map_size = 10
        world = Map(map_size)

        init_pos = np.array([0, 0])
        init_angle = 0
        turtle = Robot(world, init_pos, init_angle)

        # Initialize node
        rospy.init_node('graphics_broadcaster', anonymous=True)
        rospy.loginfo("Starting grid map!...")
        
        # Initialize simulation object
        turtleSimulation = GridWorldSimulation(turtle)

        # Main ROS loop
        rate = rospy.Rate(2)  # 10hz
        while not rospy.is_shutdown():
            
            action = np.random.randint(-1, 2)
            turtleSimulation.robot.sim_dynamics(action)
            turtleSimulation.update_robot()
            turtleSimulation.update_map()

            rospy.loginfo("Sending command: %s", action)
            rospy.loginfo("Turtle state: (%s, %s, %s)", turtleSimulation.robot.position[0], turtleSimulation.robot.position[1], turtleSimulation.robot.angle)

            rate.sleep()

    except rospy.ROSInterruptException:
        pass
