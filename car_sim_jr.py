import numpy as np
import math
import matplotlib.pyplot as plt
import tracking_util as tr


def make_noisy(x_,y_,var):
	x = x_ + np.random.normal(0,var)
	y = y_ + np.random.normal(0,var)
	
	return np.array([x,y])

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

dt = .1
c = car(0,0,0)
c.v = 1
iters = 100
positions = np.zeros((2*iters,2))
positions_track = np.zeros((2*iters,2))
x = np.array([.1,.1,.1,.1])
P = np.array([[.5,0,0,0],[0,.5,0,0],[0,0,.3,0],[0,0,0,.3]])
for i in range(iters):
	positions[i,:] = np.array([c.x,c.y])
	positions_track[i,:] = x[:2]
	x,P = tr.prediction(x,P,.5,tr.F_cv_pol)
	c.update_state(0,.5)
	z = make_noisy(c.x,c.y,.1)
	x, P = tr.correct(x,P,z,tr.H_direct_observe)
for i in range(iters):
	positions[iters + i,:] = np.array([c.x,c.y])
	positions_track[iters + i,:] = x[:2]
	x,P = tr.prediction(x,P,.5,tr.F_cv_pol)
	c.update_state(0,-.5)
	z = make_noisy(c.x,c.y,.1)
	x, P = tr.correct(x,P,z,tr.H_direct_observe)

plt.plot(positions[:,0],positions[:,1])
plt.plot(positions_track[:,0],positions_track[:,1])
plt.show()
