import numpy as np
import math

# from map import Map

class Robot:

    def __init__(self, world, initial_pos=np.zeros(2), initial_angle=0):
        
        self.world = world
        self.angle = initial_angle
        if (initial_pos[0] < world.size) and (initial_pos[1] < world.size):
            self.position = initial_pos
        else:
            self.position = np.zeros(2)

        self.last_position = self.position
        self.last_angle = self.angle

        # 1 => go_left, 0 => go_ahead, -1 => go_right
        self.last_action = 0
        self.obs_state = 0

    def sim_dynamics(self, action):

        self.last_action = action

        if action == 0:
            if self.angle == 0:
                self.position[0] += 1
            elif self.angle == 45:
                self.position[0] += 1
                self.position[1] -= 1
            elif self.angle == 90:
                self.position[1] -= 1
            elif self.angle == 135:
                self.position[0] -= 1
                self.position[1] -= 1
            elif self.angle == 180:
                self.position[0] -= 1
            elif self.angle == 225:
                self.position[0] -= 1
                self.position[1] += 1
            elif self.angle == 270:
                self.position[1] += 1
            elif self.angle == 315:
                self.position[0] += 1
                self.position[1] += 1

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

        # self.check_collision(self.position, self.angle, action)


    # def is_facing_wall(self):

    # def check_collision(self):

    # def line_of_sight(self):
