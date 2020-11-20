
#!/usr/bin/env python3

import numpy as np
import math

class Map:

	def __init__(self, size = 5):
		self.size = size
		self.map = np.zeros((self.size, self.size), dtype = np.int8)
		self.goal_position = np.zeros(2)

		# Randomize
		# self.generate_obstacles()
		self.generate_goal()

	# def generate_obstacles(self):

	def generate_goal(self):
		self.goal_position = np.array([ np.random.randint(1, high=self.size), np.random.randint(0, high=self.size) ])