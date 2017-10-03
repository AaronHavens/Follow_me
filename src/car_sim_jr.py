import numpy as np
import math
import matplotlib.pyplot as plt
import tracking_util as tr
import error_utils  as err
from math import *

def make_noisy(x_,y_,var):
	x = x_ + np.random.normal(0,var)
	y = y_ + np.random.normal(0,var)
	
	return np.array([x,y])

def PID(car, track):
	pos_e = 0

	der_e = 0

	int_e = 0

	return u_v,u_w
# Bicycle model dynamics inspired by Stanford Junior code 2008
class dyn_car():
	def __init__():
		self.x = 0
		self.y = 0
		self.vx = 0
		self.vy = 0
		self.yaw = 0
		self.yaw_rate = 0
		self.wheel_angle = 0
		self.commanded_wheel_angle = 0
		self.commanded_forward_accel = 0
		self.commanded_direction = 1
		self.actual_forward_accel = 0
		self.a = 1.37
		self.b = 1.47
		self.tire_stiffness = 145000.0
		self.mass = 2000.0
		self.iz = 5533.4531
		self.tau = 0.2
		self.steering_ratio = 1
		self.wheel_base = 7
		self.imu_to_cg_dist = 1 #maybe dont need this wheel_base + FA_bumper - L/2
		self.max_steering_rate = 10
		self.max_steering = 10
		self.direction = 1

		self.max_wheel_angle = self.max_steering*(pi/180)/self.steering_ratio
		self.max_wheel_rate = self.max_wheel_rate*(pi/180)

		self.max_brake = 100.0
		self.max_throttle = 1.0
		self.steer_inertia = 0.1
		self.brake_decel_coef = .08125
		self.throttle_accel_coef = 3.0
		self.lateral_accel = 0

	def update_state(self,dt):
		if(self.direction != self.commanded_direction):
			self.actual_forward_accel = -self.max_brake * self.brake_decel_coef
		else:
			self.actual_forward_accel += 1/.4 * (self.commanded_forward_accel-self.actual_forward_accel)*dt
		
		if(self.vx > 0):
			Fxr = 0
			if(self.direction == 1):
				Fxf = self.mass * self.actual_forward_accel
			else:
				Fxf = self.mass * -self.actual_forward_accel

			wheel_angle_dot = 1/self.tau *(self.commanded_wheel_angle-self.wheel_angle)
			self.wheel_angle_rate = wheel_angle_dot
			wheel_angle_rate_dot = 0

			if(self.vx < 5):
				alpha_f = atan2(self.vy + self.yaw_rate * self.a,5) - self.wheel_angle
				alpha_r = atan2(self.vy - self.yaw_rate * self.b,5)
			else:
				alpha_f = atan2(self.vy + self.yaw_rate * self.a,self.vx) - self.wheel_angle
				alpha_r = atan2(self.vy - self.yaw_rate * self.b,self.vx)

			Fyf = -self.tire_stiffness * alpha_f
			Fyr = self.tire_stiffness * alpha_r
			x_dot = self.vx*cos(self.yaw) - self.vy * sin(self.yaw)
			y_dot = self.vx*sin(self.yaw) + self.vy * cos(self.yaw)
			yaw_dot = self.yaw_rate
			vx_dot = 1/self.mass*(Fxr + Fxf * cos(self.wheel_angle) - Fyf * sin(self.wheel_angle)) + self.yaw_rate*self.vy
			vy_dot = 1/self.mass*(Fxr + Fxf * sin(self.wheel_angle) + Fyf * cos(self.wheel_angle)) - self.yaw_rate*self.vx
			yaw_rate_dot = 1/self.iz*(self.a * Fxf * sin(self.wheel_angle) + self.a*Fyf*cos(self.wheel_angle)-self.b*Fyr)
		#### simple bicycle model for 0 or less than speed?
		else:
			wheel_angle_dot = 1 /self.tau * (self.commanded_wheel_angle - self.wheel_angle)
			self.wheel_angle_rate = wheel_angle_dot
			wheel_angle_rate_dot = 0

			x_dot = self.vx * cos(self.yaw)
			y_dot = self.vx * sin(self.yaw)
			yaw_dot = self.vx * tan(self.wheel_angle)/self.wheel_base

			if(direction == 0):
				self.actual_forward_accel *= -1

			vx_dot = self.actual_forward_accel
			vy_dot = 0
			yaw_rate_dot = 0

		####
		self.x += x_dot*dt
		self.y += y_dot*dt
		self.yaw += yaw_dot*dt
		self.vx += vx_dot *dt
		if(direction == 1 and self.vx < 0):
			self.actual_forward_accel = 0
			self.vx = 0
		elif(direction == 0 and self.vx > 0):
			self.actual_forward_accel = 0 
			self.vx = 0

		self.vy += vy_dot*dt
		self.yaw_rate += yaw_rate_dot * dt
		if(self.vx == 0):
			self.vy = 0
			self.yaw_rate = 0

		if(self.wheel_angle > self.max_steering/self.steering_ratio):
			self.wheel_angle = self.max_steering/self.steering_ratio
			if(self.wheel_angle_rate > 0):
				self.wheel_angle_rate = 0
			if(wheel_angle_dot > 0):
				wheel_angle_dot = 0
			if(wheel_angle_rate_dot > 0):
				wheel_angle_rate_dot = 0
		elif(self.wheel_angle < -self.max_steering/self.steering_ratio):
			self.wheel_angle = -self.max_steering/self.steering_ratio
			if(self.wheel_angle_rate < 0):
				self.wheel_angle_rate = 0
			if(wheel_angle_dot < 0):
				wheel_angle_dot = 0
			if(wheel_angle_rate_dot < 0):
				wheel_angle_rate_dot = 0

		self.wheel_angle += wheel_angle_dot*dt
		self.wheel_angle_rate += wheel_angle_rate_dot*dt

		self.lateral_accel = vy_dot




	def set_controls(self,steering_angle, throttle_fraction, brake_pressure):
		print('do nothing')



class car():
	def __init__(self,x,y,theta):
		self.x = x
		self.y = y
		self.v = 0
		self.theta = theta
		self.v_max = 10
		self.u_v_max = 10
		self.u_w_max = .5
		self.L = 1.0
		
	def update_state(self,u_v,u_w,dt):
		
		if(abs(u_v) > self.u_v_max):
			u_v = self.u_v_max*u_v/(abs(u_v))
		if(abs(u_w) > self.u_w_max):
			u_w = self.u_w_max*u_w/(abs(u_w))
		self.theta += self.v/self.L*u_w*dt

		self.v += u_v*dt	

		if(abs(self.v) > self.v_max):
			self.v = self.v_max*self.v / abs(self.v)
		
		
		self.x += self.v*cos(self.theta)*dt
		self.y += self.v*sin(self.theta)*dt
	
## TEST BLOCK
dt = 1
c = car(0,0,0)
c.v = 1
iters = 20
positions = np.zeros((iters,2))
positions_track = np.zeros((iters,2))
x = np.array([.01,.01,.01,1.3])
P = np.array([[.5,0,0,0],[0,.5,0,0],[0,0,.3,0],[0,0,0,.3]])

for i in range(iters):
	positions[i,:] = np.array([c.x,c.y])
	positions_track[i,:] = x[:2]
	x,P = tr.prediction(x,P,dt,tr.F_cv_pol)
	c.update_state(.1,.1,dt)
	z = make_noisy(c.x,c.y,.05)
	x, P = tr.correct(x,P,z,tr.H_direct_observe)

theta = err.poly_3_fit(positions[:6,:])
traj = err.gen_traj(positions[:6,0],theta)

plt.scatter(positions[:,0],positions[:,1])
plt.plot(traj[:,0],traj[:,1],color = 'r')

#plt.plot(positions_track[:,0],positions_track[:,1])
plt.show()
