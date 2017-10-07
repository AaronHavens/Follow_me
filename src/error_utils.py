import numpy as np


def poly_3_fit(track_xy):
	x = track_xy[:,0]
	y = track_xy[:,1]
	F = np.zeros((len(x),4))
	for i in range(len(x)):
		F[i,:] = np.array([x[i]**3,x[i]**2,x[i],1])

	F_T = np.transpose(F)
	Psuedo_Inv = np.linalg.inv(np.dot(F_T,F))
	Res = np.dot(F_T,y)
	theta = np.dot(Psuedo_Inv,Res)

	return theta # [a,b,c,d] --> a*x^3 + b*x^2 + c*x + d

def gen_traj(x,theta):
	traj = np.zeros((len(x),2))
	traj[:,0] = x
	for i in range(len(x)):
		traj[i,1] = theta[0]*x[i]**3 + theta[1]*x[i]**2 + theta[2]*x[i] + theta[3]
	return traj

def d_x_traj(theta,x):
	print(theta)
	a = theta[0]
	b = theta[1]
	c = theta[2]
	return (3*a*x**2 + 2*b*x + c) #gradient

def ot_e_test(traj_x,vehicle_pos):
	print('do nothing')

