import cvxopt as cvx
import numpy as np
import math


#constant speed value for now

#return jacobians for ackerman steering kinematics given a reference center
def ackerman_model(state_center, input_center, delta_time,vf=5.0):
	

	delta_ref  = input_center[0]
	v_f = state_center[2]
	theta_ref = state_center[3]

	A = np.array([[1,0,-v_f*delta_time*math.sin(theta_ref+delta_ref)],
				[0,1,v_f*delta_time*math.cos(theta_ref + delta_ref)],
				[0,0,1]])
	B = np.array([[-vf*delta_time*math.sin(theta_ref + delta_ref)],
				[vf*delta_time*math.cos(theta_ref+delta_ref)],
				[vf*delta_time/1.0*math.cos(delta_ref)]])
	return A, B



class MPC_controller(object):

	def __init__(self,dynamics, state_0, u_min, u_max, u_cost, horizon,track_weighting):
		
		self.dynamics = dynamics
		self.u_min = u_min
		self.u_max = u_max
		self.u_cost = u_cost
		self.track_weighting = track_weighting
		self.horizon = horizon

		#P, H = self.construct_pred_mat(state_0)
        self.P = P
        self.H = H


    def construct_pred_mat(self,ref_traj, ref_inputs):
    	a_dim = len(ref_traj[0])
    	b_dim = len(ref_inputs[0])
    	#initialize matricies
    	P = np.zeros((a_dim*horizon,a_dim))
    	H = np.zeros((a_dim*horizon,b_dim*horizon))

    	As = np.zeros(horizon)
    	Bs = np.zeros(horizon)
    	#last_A = np.zeros((a_dim,a_dim))
    	for i in range(horizon):
    		index_a = i*a_dim
    		index_b = i*b_dim

    		As[i], Bs[i] = self.dynamics(ref_traj[i],ref_inputs[i])
    		
    	alpha = np.zeros((a_dim*horizon,a_dim*horizon))

    	for i in range(horizon):
    		A_sum = None
    		for j in range(i, horizon):
    			if not A_sum:
    				A_sum = As[k]
    			else:
    				A_sum = np.dot(As[j],A_sum)
    			l=horizon-i
    			alpha[i*a_dim:i*a_dim+a_dim,l*a_dim:l*a_dim+a_dim] = A_sum
    	for i in reversed(1,range(horizon)):
    		P[i*a_dim:i*a_dim+a_dim,:] = alpha[0,i]

    	i_index = 0
    	j_index = 0 
    	for i in range(horizon):
    		j = horizon
    		for j in range(i):
    			H[i*b_dim:i*b_dim+b_dim,j*a_dim:j*a_dim+a_dim] = np.dot(alpha[i_index,j_index],Bs[j_index])
    			j_index -= 1
    		if i == j: H[i*b_dim:i*b_dim+b_dim,j*a_dim:j*a_dim+a_dim] = Bs[j_index]
    		i_index += 1

    	return P, H

	def compute_linearized_dyn(self, ref_traj):
		P, H = self.pred_mat(ref_traj)
		self.P = P
		self.H = H

def construct_trajectory(horizon):
	ref_traj = np.zeros(horizon)
	ref_inputs = np.zeros(horizon)
	for i in range(horizon):
		ref_traj[i] = 0.0
		ref_inputs[i] = 0.0 + np.uniform(-0.05, 0.05)

state = np.array([5.0,1.0,0.0])

horizon = 5
u_min = -2.0
u_max = 2.0
u_cost = 1.0
track_weighting = 0



controller = MPC_controller(ackerman_model,state, u_min, u_max, u_cost, horizon,track_weighting)





