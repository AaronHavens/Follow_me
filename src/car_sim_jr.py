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

def pursuit_turn_angle(car_pos, target):
	dx = target[0] - car_pos[0]
	dy = target[1] - car_pos[1]
	alpha = atan2(dy,dx)
	l = sqrt(dx*dx + dy*dy)
	print(l)
	steer = -2*dx/(l*l) #output steering radius to join target
	return steer

def PID_test(traj_x,car):
	ct_e = traj_x - car.x
	ct_e_d = ct_e/abs(ct_e)*car.v*cos(car.theta)
	p = .1
	d = 1/5
	return  -(p*ct_e + d*ct_e_d)

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
		self.steer = 0
		self.throttle = 0
		self.v_max = 10
		self.u_a_max = 10
		self.u_steer_max = .5
		self.L = 1.0
		
	def update_state(self,u_a,u_w,dt):
		
		if(abs(u_a) > self.u_a_max):
			u_a = self.u_a_max*u_v/(abs(u_a))
		if(abs(u_w) > self.u_steer_max):
			u_w = self.u_steer_max*u_w/(abs(u_w))
		self.steer = u_w
		self.theta += self.v/self.L*self.steer*dt

		self.v += u_a*dt	

		if(abs(self.v) > self.v_max):
			self.v = self.v_max*self.v / abs(self.v)
		
		
		self.x += self.v*cos(self.theta)*dt
		self.y += self.v*sin(self.theta)*dt
	
## TEST BLOCK
dt = .1
t_t = 0
c = car(0,0,pi/2)
c.v = 1
iters = 1000
positions = np.zeros((iters,2))
positions_track = np.zeros((iters,2))
positions_track_ref = np.zeros((iters,2))
x = np.array([12.01,.01,pi/2+.1,5])
P = np.array([[.5,0,0,0],[0,.5,0,0],[0,0,.3,0],[0,0,0,.3]])

steer = pursuit_turn_angle([0,0,0,pi/2],[10,20])
print(steer)
dt = 1
path = np.zeros((20,2))
for i in range(1,20):
	c.update_state(0,steer,dt)
	path[i,:] = [c.x,c.y]
plt.plot(path[:,0],path[:,1],'--r')
plt.scatter(10,20)
# t = np.zeros(iters)
# for i in range(iters):
# 	t_t += dt
# 	t[i] = t_t
# 	positions[i,:] = np.array([c.x,c.y])
# 	positions_track[i,:] = x[:2]
# 	x,P = tr.prediction(x,P,dt,tr.F_cv_pol)
# 	steer = PID_test(traj_x,c)
# 	c.update_state(0,steer,dt)
# 	z = make_noisy(c.x,c.y,.45)
# 	positions_track_ref[i,:] = z
# 	x, P = tr.correct(x,P,z,tr.H_direct_observe)
# 	#if(i%20 == 0):
# 		#plt.arrow(x[0],x[1], x[3]*cos(x[2]), x[3]*sin(x[2]), head_width=0.1, head_length=0.1, fc='k', ec='k')

# track_segs = err.param_fit(positions_track,t,40)
# ax = plt.axes()
# ax.plot(track_segs[:,0],track_segs[:,1])

# ax.plot([0,0],[0,100],'--r')
# ax.scatter(positions_track_ref[:,0],positions_track_ref[:,1], s=.5)
# ax.scatter(positions_track[:,0],positions_track[:,1],s=.8)
plt.axis('equal')
plt.show()
