import numpy as np
import math


class Robot:

    def __init__(self, world, initial_pos=np.zeros(2), initial_angle=0):

        self._world = world
        self._angle = initial_angle
        if (initial_pos[0] < world.get_size()) and (initial_pos[1] < world.get_size()):
            self._position = initial_pos
        else:
            self._position = np.zeros(2)

        self._last_position = self._position
        self._last_angle = self._angle

        # -1 => go_left, 0 => go_ahead, 1 => go_right
        self._last_action = 0
        self._obs_state = 0

    def sim_dynamics(self, action):

        self._last_action = action

        if action == 0:
            if self._angle == 0:
                self._position[0] += 1
            elif self._angle == 45:
                self._position[0] += 1
                self._position[1] += 1
            elif self._angle == 90:
                self._position[0] += 1
            elif self._angle == 135:
                self._angle -= 1
                self._position[0] += 1
            elif self._angle == 180:
                self._position[0] -= 1
            elif self._angle == 225:
                self._position[0] = -1
                self._position[0] = -1
            elif self._angle == 270:
                self._position[0] -= 1
            elif self._angle == 315:
                self._position[0] += 1
                self._position[0] -= 1

        if action == -1:
            if self._angle == 0:
                self._angle = 45
            elif self._angle == 45:
                self._angle = 90
            elif self._angle == 90:
                self._angle = 135
            elif self._angle == 135:
                self._angle = 180
            elif self._angle == 180:
                self._angle = 225
            elif self._angle == 225:
                self._angle = 270
            elif self._angle == 270:
                self._angle = 315
            elif self._angle == 315:
                self._angle = 0

        if action == 1:
            if self._angle == 0:
                self._angle = 315
            elif self._angle == 45:
                self._angle = 0
            elif self._angle == 90:
                self._angle = 45
            elif self._angle == 135:
                self._angle = 90
            elif self._angle == 180:
                self._angle = 135
            elif self._angle == 225:
                self._angle = 180
            elif self._angle == 270:
                self._angle = 225
            elif self._angle == 315:
                self._angle = 270

        self.check_collision(self._position, self._angle, action)


    def is_facing_wall(self):


    def check_collision(self):


        return 1

    def line_of_sight(self):
