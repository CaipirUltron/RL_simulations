#!/usr/bin/env python3

import numpy as np
import math

# from map import Map

class Robot:

    def __init__(self, world, initial_pos=np.zeros(2,dtype=np.int8), initial_angle=0):
        
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
            print("Initial position is not a free space!")

        # Possible actions: 1 => go_left, 0 => go_ahead, -1 => go_right
        self.last_action = 0
        self.obs_state = 0

    def sim_dynamics(self, action):

        # GO AHEAD action
        if action == 0:

            if self.angle == 0:
                self.next_position = np.array([ np.minimum(self.next_position[0] + 1,self.world.size-1), self.next_position[1] ])
            elif self.angle == 45:
                self.next_position = np.array([ np.minimum(self.next_position[0] + 1,self.world.size-1), np.maximum(self.next_position[1] - 1,0) ])
            elif self.angle == 90:
                self.next_position = np.array([ self.next_position[0], np.maximum(self.next_position[1] - 1,0) ])
            elif self.angle == 135:
                self.next_position = np.array([ np.maximum(self.next_position[0] - 1,0), np.maximum(self.next_position[1] - 1,0) ])
            elif self.angle == 180:
                self.next_position = np.array([ np.maximum(self.next_position[0] - 1,0), self.next_position[1] ])
            elif self.angle == 225:
                self.next_position = np.array([ np.maximum(self.next_position[0] - 1,0), np.minimum(self.next_position[1] + 1,self.world.size-1) ])
            elif self.angle == 270:
                self.next_position = np.array([ self.next_position[0], np.minimum(self.next_position[1] + 1,self.world.size-1) ])
            elif self.angle == 315:
                self.next_position = np.array([ np.minimum(self.next_position[0] + 1,self.world.size-1), np.minimum(self.next_position[1] + 1,self.world.size-1) ])

            # Collision check: if robot has collided, then the position is not updated.
            if not self.check_collision():
                self.position = self.next_position
            else:
                self.next_position = self.position
                print("Collision!")

        # print("Actual Pos = ", self.position)
        # print("Next Pos = ", self.next_position)

        if action == 1:
            if self.angle != 315:
                self.angle += 45
            else:
                self.angle = 0

        if action == -1:
            if self.angle != 0:
                self.angle -= 45
            else:
                self.angle = 315

    def check_collision(self):

        return self.check_wall_collision() or self.check_obstacle_collision()

    def check_wall_collision(self):

        if (self.next_position[0] >= self.world.size) or (self.next_position[0] < 0) or (self.next_position[1] >= self.world.size) or (self.next_position[1] < 0): # reached a wall
            return True
        else:
            return False

    def check_obstacle_collision(self):

        if (self.world.map[self.next_position[1]][self.next_position[0]] == 1):
            return True
        else:
            return False

    # def line_of_sight(self):  