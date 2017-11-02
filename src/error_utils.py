import numpy as np


def param_fit(track_xy,t,partition): #produces smooth peicewise trajectory to follow
	partitions = np.size(track_xy[:,0])//partition + 1# track every 10 points
	remain = np.size(track_xy[:,0])%partition
	traj_list = np.zeros((np.size(track_xy[:,0]),2))# x,y,heading_x,heading_y estimated.
	index = 0
	for k in range(partitions):
		print(k)
		if(k < partitions - 1):
			offset = partition
		elif(remain > 1):
			offset = remain
		else:
			break
		track_xy_k = track_xy[index:index+offset,:]
		t_k = t[index:index+offset]
		x = track_xy_k[:,0]
		y = track_xy_k[:,1]
		F = np.zeros((2*len(x),6))
		y_z = np.zeros((2*len(x),1))
		for i in range(len(x)):
			F[2*i:2*i+2,:] = np.array([[1,0,t_k[i],0,t_k[i]**2,0],[0,1,0,t_k[i],0,t_k[i]**2]])
			y_z[2*i,0] = x[i]
			y_z[2*i+1,0] = y[i]
		F_T = np.transpose(F)
		Psuedo_Inv = np.linalg.inv(np.dot(F_T,F))
		Res = np.dot(F_T,y_z)
		theta = np.dot(Psuedo_Inv,Res)
		traj_list[index:index + offset,:] = gen_traj(theta,t_k)
		index += partition
		
	return traj_list # [a,b,c,d] --> a*x^3 + b*x^2 + c*x + d

def gen_traj(theta,t):
	traj = np.zeros((len(t),2))
	for i in range(len(t)):
		traj[i,0] = theta[0] + theta[2]*t[i] + theta[4]*t[i]**2
		traj[i,1] = theta[1] + theta[3]*t[i] + theta[5]*t[i]**2
	return traj

def d_x_traj(theta,x):
	print(theta)
	a = theta[0]
	b = theta[1]
	c = theta[2]
	y1 = 3*a*x**2 + 2*b*x + c #gradient
	y2 = 3*a*(x+1)**2 + 2*b*(x+1) + c
	norm = np.sqrt(1 + (y2 - y1)**2)
	return [(y2-y1)/norm , 1/norm]

def ot_e_test(traj_x,vehicle_pos):
	print('do nothing')

def carrot(car,d):
	carrot_x = vehicle.x + d*cos(car.theta)
	carrox_y = vehicle.y + d*sin(car.theta)
	return [carrot_x, carrox_y]

def e_y(car, carrot, traj):
	return 0
	#x = diff(traj,proj)

