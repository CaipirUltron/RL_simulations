
from robot import Robot
from map import Map

map_length = 5
gridWorld = Map(map_length)

initial_pos = np.array([ 2 2 ])
initial_angle = 45

turtle = Robot(gridWorld, initial_pos, initial_angle)