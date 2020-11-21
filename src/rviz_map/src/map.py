
#!/usr/bin/env python3

import numpy as np
import math

class Map:

	def __init__(self, size = 5, map = None, goal = np.zeros(2)):

		self.size = size
		if map:
			self.map = map
		else:
			self.map = np.zeros((self.size,self.size),dtype = np.int8)
		self.goal_position = goal
		self.occupancy_p = 0.0

	# Generate random goal position
	def generate_random_goal(self):

		if (self.occupancy_p == 1.0):
			self.goal_position = np.array([0, self.size*self.size])
			print("No free spaces available.")
		else:
			self.goal_position = np.random.randint(0, self.size-1, size=2)
			while self.check_goal_collision():
				self.goal_position = np.random.randint(0, self.size-1, size=2)
				print("Goal is colliding with obstacle. Resetting goal position...")

	# Generate random map with occupancy probability in the interval [0.0,1.0]
	def generate_random_map(self, p = 0.5):

		self.occupancy_p = p
		for i in range(0,self.size):
			for j in range(0,self.size):
				self.map[i,j] = np.random.choice(np.arange(0,2), p = [1-self.occupancy_p, self.occupancy_p])

	# Checks if goal is inside an obstacle
	def check_goal_collision(self):
		if self.map[self.goal_position[1],self.goal_position[0]] == 1:
			return True
		else:
			return False