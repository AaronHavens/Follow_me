import numpy as np
import math
import matplotlib.pyplot as plt


dt = .01
def PID(car, track):

	pos_e = 0

	der_e = 0

	int_e = 0

	return u_v,u_w
class car():


	def __init__(self,x,y,theta):
		self.x = x
		self.y = y
		self.v = 0
		self.theta = theta
		self.w = 0
		self.v_max = 10
		self.w_max = math.pi/4
		self.u_v_max = 10
		self.u_w_max = .5

	def update_state(self,u_v,u_w):
		global dt
		if(abs(u_v) > self.u_v_max):
			u_v = self.u_v_max * u_v/(abs(u_v))
		if(abs(u_w) > self.u_w_max):
			u_w = self.u_w_max * u_w/(abs(u_w))

		self.w += u_w*dt
		self.v += u_v*dt

		if(abs(self.w) > self.w_max):
			self.w = self.w_max * self.w / abs(self.w)
		if(abs(self.v) > self.v_max):
			self.v = self.v_max * self.w / abs(self.w)

		self.theta += u_w*dt
		vx = math.cos(self.theta)*self.v
		vy = math.sin(self.theta)*self.v
		self.x += vx*dt
		self.y += vy*dt

c = car(0,0,0)
c.v = 1
iters = 1000
positions = np.zeros((2*iters,2))

for i in range(iters):
	positions[i,:] = np.array([c.x,c.y])
	c.update_state(0,.5)
for i in range(iters):
	positions[1000 + i,:] = np.array([c.x,c.y])
	c.update_state(0,-.5)

plt.scatter(positions[:,0],positions[:,1])
plt.show()