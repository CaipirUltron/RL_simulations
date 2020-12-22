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
from qlearning import QLearning



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
        self.goal_marker.type = self.goal_marker.CYLINDER
        self.goal_marker.action = self.goal_marker.ADD
        self.goal_marker.scale.x = 1.0
        self.goal_marker.scale.y = 1.0
        self.goal_marker.scale.z = 0.1
        self.goal_marker.color.a = 1.0
        self.goal_marker.color.r = 1.0
        self.goal_marker.color.g = 0.0
        self.goal_marker.color.b = 0.0
        self.goal_marker.pose.position.x = 0.0
        self.goal_marker.pose.position.y = 0.0
        self.goal_marker.pose.position.z = -0.2
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

        # Sends map transformation and publishes OccupancyGrid
        quat_map = quaternion_from_euler(np.radians(180), 0, 0)
        self.world_tf.sendTransform((0,0,0),quat_map,rospy.Time.now(),"map_frame", "world")
        self.map_publisher.publish(self.rviz_map)

    # This function receives a Pose ROS msg and updates the corresponding transformation
    def update_robot(self):

        robot_marker_position_x = -self.map_size/2 + self.robot.position[0] + 0.5
        robot_marker_position_y = self.map_size/2 - self.robot.position[1] - 0.5
        robot_quat = quaternion_from_euler(0, 0, np.radians(self.robot.angle))

        # Sends transformations and publishes Rviz graphical objects
        self.world_tf.sendTransform((robot_marker_position_x,robot_marker_position_y,0),robot_quat,rospy.Time.now(),"robot_frame", "world")
        self.robot_marker_publisher.publish(self.robot_position_marker)
        self.angle_marker_publisher.publish(self.robot_angle_marker)

    # This function receives a Point ROS msg and updates the corresponding transformation
    def update_goal(self):

        goal_position = self.robot.world.goal_position
        robot_marker_position_x = -self.map_size/2 + goal_position[0] + 0.5
        robot_marker_position_y = self.map_size/2 - goal_position[1] - 0.5

        # Sends transformation and publishes Rviz graphical objects
        self.world_tf.sendTransform((robot_marker_position_x,robot_marker_position_y,0),(0,0,0,1),rospy.Time.now(),"goal_frame", "world")
        self.goal_marker_publisher.publish(self.goal_marker)

if __name__ == '__main__':
    try:
        
        # Define map and goal position
        map_size = 6
        occupancy_probability = 0.4        #controls probability of obstacles
        goal_position = np.array([2 , 2])   #TEST CONDITION
        world = Map(map_size, goal_position)   #TEST CONDITION
        #world = Map(map_size)          #REPOR APOS TESTE
        world.generate_random_map(occupancy_probability)
        world.generate_random_goal()

        # Define robot
        init_pos = np.array([2, 0])
        init_angle = 0
        turtle = Robot(world, init_pos, init_angle)    

        #Define      

        # Initialize node
        rospy.init_node('graphics_broadcaster', anonymous=True)
        rospy.loginfo("Starting grid map!...")
        
        # Initialize simulation object
        turtleSimulation = GridWorldSimulation(turtle)

        # Main ROS loop
        rate = rospy.Rate(10000)  # 1000hz







        # Define QLearning
        Q_var = QLearning(turtle)
        epsilon = 0.5               #determina exploration ou exploitation
        Q_var.current_state()

        moves=0
        episode = 1

        while not rospy.is_shutdown():
            
            # Updates map graphics
            turtleSimulation.update_map()








            if np.random.uniform(0, 1) < epsilon:
                action = turtleSimulation.robot.go_random()                          # Explore action space
                Q_var.action_taken = action             #para saber qual action tomou
            else:
                Q_var.choose_action()                                                # Exploit learned values


            Q_var.next_state()

           

            Q_var.update_Qvalue()



            Q_var.current_state()                               #atualiza estado


            # Updates robot and goal graphics
            turtleSimulation.update_robot()
            turtleSimulation.update_goal()

            rospy.loginfo("Line of sight: (%s,%s,%s)", turtleSimulation.robot.observation[0], turtleSimulation.robot.observation[1], turtleSimulation.robot.observation[2])

            rate.sleep()

            moves += 1

            

            if(moves == 1000):                  #se estiver stuck no episodio faz reset
                world.generate_random_map(occupancy_probability)
                world.generate_random_goal()
                turtle = Robot(world, init_pos, init_angle)   
                moves=1
                episode+=1
                Q_var.current_state() 

            if(episode==50):                  #visualizar resultado do qlearning
                epsilon = 0
                rate = rospy.Rate(1)  # 1hz

            if(episode==100):                  #quando acabar episodios
                break
            
            print("episode: ", episode)

        print(Q_var.Q_table)                    #imprime tabela no fim
    except rospy.ROSInterruptException:
        pass
