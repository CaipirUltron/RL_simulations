#!/usr/bin/env python3

import numpy as np
import math

# from map import Map


class Robot:

    def __init__(self, world, initial_pos=np.zeros(2, dtype=np.int8), initial_angle=0):

        self.world = world
        self.angle = initial_angle

        self.next_position = initial_pos
        self.position = initial_pos

        if self.check_wall_collision():
            print("Initial position outside of map bounds!")
            new_pos = np.random.randint(0, 10, size=2)
            self.next_position = new_pos
            self.position = new_pos

        if self.check_obstacle_collision():
            print("Initial position is not a free space.")

        # Possible actions: 1 => go_left, 0 => go_ahead, -1 => go_right
        self.obs_array = np.zeros(3)

    def sim_dynamics(self, action):

        # GO AHEAD action
        if action == 0:

            direction = self.get_forward()
            self.next_position = self.position + direction

            # Collision check: if robot has collided, then the position is not updated.
            if not self.check_collision():
                self.position = self.next_position
                print("Free move")
            else:
                print("Collision")

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

    # def get_neighbors(self):
        

    def get_observation(self):

        if self.angle == 0:
            self.obs_array[1] = self.world.map[self.position[1]][self.position[0]]

    # Check general collisions
    def check_collision(self):

        if self.check_wall_collision():
            return True
        elif self.check_obstacle_collision():
            return True
        else:
            return False

    # Check collisions against a wall
    def check_wall_collision(self):

        if (self.next_position[0] >= self.world.size) or (self.next_position[0] < 0) or (self.next_position[1] >= self.world.size) or (self.next_position[1] < 0): # reached a wall
            return True
        else:
            return False

    # Check collisions against an obstacle on the map
    def check_obstacle_collision(self):

        if (self.world.map[self.next_position[1],self.next_position[0]] == 1):
            return True
        else:
            return False

    # Check if the goal was reached
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

    # Perform random movement
    def go_random(self):
        self.sim_dynamics(self.random_action())

    # Returns random action
    @staticmethod
    def random_action():
        return np.random.randint(-1, 2)

    # def line_of_sight(self):
