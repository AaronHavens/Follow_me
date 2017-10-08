import numpy as np
from math import *





#class tracker():
#	
#	def __init__(self,init):
#		self.state = init
#		self.

#Use if your state is [x,y,vx,vy]
# x_t+1 = x_t + vx*dt
def F_cv_rect(x_t_t,dt):
	F = np.array([[1,0,dt,0],[0,1,0,dt],[0,0,1,0],[0,0,0,1]])
	return F

#Use if your state is [x,y,heading,speed]
# x_t+1 = x_t + v*cos(theta)*dt
#Notice that I need to use the current state to linearize about. Not a linear transition in rect space.
def F_cv_pol(x_t_t,dt):
	theta,v = x_t_t[2:] 
	F = np.array([[1,0,-sin(theta)*v*dt,cos(theta)*dt],[0,1,cos(theta)*v*dt,sin(theta)*dt],[0,0,1,0],[0,0,0,1]])
	return F
def f_pol(x_t,dt):
	x = x_t[0] + x_t[3]*cos(x_t[2])*dt
	y = x_t[1] + x_t[3]*sin(x_t[2])*dt
	return np.array([x,y,x_t[2],x_t[3]])
#Observation function assumes complete observability. Very Naive
def H_direct_observe(x_t):
	H = np.array([[1.0,0,0,0],[0,1.0,0,0]])
	h = np.array([x_t[0],x_t[1]])
	return h,H

#Standard white noise formulation
def white_noise_process(var,dt):
	#arbitrary noise as a linear function of time
	dt_2 = dt*dt
	dt_3 = dt_2*dt
	dt_4 = dt_3*dt
	dt_4_4 = dt_4 / 4
	dt_3_2 = dt_3 / 2
	Q = np.array([[dt_4_4*var,0,dt_3_2*var,0],[0,dt_4_4*var,0,dt_3_2*var],[dt_3_2*var,0,dt_2*var,0],[0,dt_3_2*var,0,dt_2*var]])
	return Q
#Dumb measurement proccess noise to avoid 0 variance
def white_noise_measurement(dt):
	V = np.array([[2.0*dt,0],[0,2.0*dt]])
	return V

#Pass in desired transition function, state and time difference to propogate state forward
def prediction(x_t,P_t,dt,F_function):
	F = F_function(x_t,dt)
	
	x_new = f_pol(x_t,dt)

	P_new = np.dot(F,P_t)
	P_new = np.dot(P_new,np.transpose(F)) + white_noise_process(.1,dt)
	return x_new, P_new

#Perform correction step with new measurement. Supports alternate measurement functions
def correct(x_t,P_t,z,H_function):
	h, H = H_function(x_t)
	
	y = z - h
	
	S = np.dot(H,P_t)
	S = np.dot(S,np.transpose(H)) + white_noise_measurement(.005)
	K = np.dot(P_t,np.transpose(H))
	K = np.dot(K,np.linalg.inv(S))
	
	I = np.identity(np.size(x_t))

	x_new = x_t + np.dot(K,y)

	grad = np.dot(K,H)
	P_new = np.dot((I-grad),P_t)
	
	return x_new, P_new

