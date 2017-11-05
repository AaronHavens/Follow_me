import numpy as np
from math import *

def pursuit_turn_angle(car_pos, target):
	dx = target[0] - car[0]
	dy = target[1] - car[1]

	l = sqrt(dx*dx + dy*dy)

	steet = 2*dx/(l*l) #output steering radius to join target