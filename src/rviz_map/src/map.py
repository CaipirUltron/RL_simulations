import numpy as np
import math


class Map:

	def __init__(self, size = 5):
		self._size = size
		self._map = np.zeros((self._size, self._size), dtype = np.int8)
		self._goal_position = np.zeros(2)

		# Randomize
		# self.generate_obstacles()
		self.generate_goal()

	# def generate_obstacles(self):

	def get_size(self):
		return self._size

	def generate_goal(self):
		self._goal_position = np.array([ np.random.randint(1, high=self._size), np.random.randint(0, high=self._size) ])