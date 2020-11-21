#!/usr/bin/env python3

import numpy as np


class Robot:

    def __init__(self, world, initial_pos=np.zeros(2, dtype=np.int8), initial_angle=0):

        self.world = world
        self.angle = initial_angle

        self.next_position = initial_pos

        # Check if robot is initially inside the map 
        if self.check_wall_collision():
            print("Initial position outside of map bounds. Reinitializing with random valid position...")
            self.next_position = np.random.randint(0, self.world.size, size=2)

        if self.check_obstacle_collision():
            print("Initial position is not a free space. Resetting initial robot position...")
            self.generate_random_position()

        # Updates robot initial position and gets initial neighbor occupancies
        self.position = self.next_position

        # Updates the robot's observation of nearby obstacles
        self.neighbors = np.zeros(8)    # occupancy values of all 8 adjacent cells around the robot
        self.observation = np.zeros(3)  # occupancy values of the 3 cells in front of the robot
        self.get_neighbors()

    # Generates random next position for the robot
    def generate_random_position(self):

        if (self.world.occupancy_p == 1.0):
            raise Exception("No free spaces available on the map.")
        else:
            self.next_position = np.random.randint(0, self.world.size, size=2)
            while self.check_obstacle_collision():
                self.next_position = np.random.randint(0, self.world.size, size=2)

    # Computes robot dynamics. Possible actions: 0 => go_ahead, 1 => go_left, -1 => go_right
    def sim_dynamics(self, action):

        # GO AHEAD action
        if action == 0:

            direction = self.get_forward()
            self.next_position = self.position + direction

            # Collision check: if robot has collided, then the position is not updated.
            if not self.check_collision():
                self.position = self.next_position

            # If the robot reached the goal position, then generate a new one
            if self.check_goal_reached():
                self.world.generate_random_goal()

        # GO LEFT action
        if action == 1:
            if self.angle != 315:
                self.angle += 45
            else:
                self.angle = 0

        # GO RIGHT action
        if action == -1:
            if self.angle != 0:
                self.angle -= 45
            else:
                self.angle = 315

        # Update neighbors
        self.get_neighbors()

    # This function populates an 8-dim array containing the occupancies of the 8 cells adjacent to the robot.
    # The first element of self.neighbors always corresponds to the occupancy of the cell in front of the robot.
    # The remaining elements contain the occupancy of the remaining cells sorted in the counter-clockwise direction.
    def get_neighbors(self):

        # Define augmented map with obstacles on the borders
        aug_map = np.ones((self.world.size+2,self.world.size+2),dtype=np.int8)
        aug_map[1:-1,1:-1] = self.world.map

        # Get new robot positions on the augmented map
        new_i_pos = self.position[1] + 1
        new_j_pos = self.position[0] + 1

        # Get robot surronding occupancy window
        window = aug_map[new_i_pos-1:new_i_pos+2,new_j_pos-1:new_j_pos+2]

        # Populate observation vector with all 8 neighbors
        observation = np.zeros(8,dtype=np.int8)
        observation[0] = window[1,2]
        observation[1] = window[0,2]
        observation[2] = window[0,1]
        observation[3] = window[0,0]
        observation[4] = window[1,0]
        observation[5] = window[2,0]
        observation[6] = window[2,1]
        observation[7] = window[2,2]

        # Rotates observation vector to fit robot's orientation
        obs_roll = -int(self.angle/45)
        self.neighbors = np.roll(observation, obs_roll)

        # Gets the values of the cells right in front of the robot
        self.observation = np.array([self.neighbors[1], self.neighbors[0], self.neighbors[-1]])

    # Returns line of sight observation
    def line_of_sight(self):

        return self.observation

    # Checks for general collisions
    def check_collision(self):

        if self.check_wall_collision():
            return True
        elif self.check_obstacle_collision():
            return True
        else:
            return False

    # Checks for collisions against a wall
    def check_wall_collision(self):

        if (self.next_position[0] >= self.world.size) or (self.next_position[0] < 0) or (self.next_position[1] >= self.world.size) or (self.next_position[1] < 0): # reached a wall
            return True
        else:
            return False

    # Checks for collisions against an obstacle on the map
    def check_obstacle_collision(self):

        if (self.world.map[self.next_position[1],self.next_position[0]] == 1):
            return True
        else:
            return False

    # Checks if the robot has reached the goal
    def check_goal_reached(self):

        if self.position[0] == self.world.goal_position[0] and self.position[1] == self.world.goal_position[1]:
            return True
        else:
            return False

    # Returns the forward direction vector, depending on orientation
    def get_forward(self):
        
        if self.angle == 0:
            direction = np.array([1,0])
        elif self.angle == 45:
            direction = np.array([1,-1])
        elif self.angle == 90:
            direction = np.array([0,-1])
        elif self.angle == 135:
                direction = np.array([-1,-1])
        elif self.angle == 180:
            direction = np.array([-1,0])
        elif self.angle == 225:
            direction = np.array([-1,1])
        elif self.angle == 270:
            direction = np.array([0,1])
        elif self.angle == 315:
            direction = np.array([1,1])

        return direction

    # Go forward
    def go_forward(self):
        self.sim_dynamics(0)

    # Go left
    def go_left(self):
        self.sim_dynamics(1)

    # Go right
    def go_right(self):
        self.sim_dynamics(-1)

    # Performs random movement
    def go_random(self):
        self.sim_dynamics(self.random_action())

    # Returns random action
    @staticmethod
    def random_action():
        return np.random.randint(-1, 2)
