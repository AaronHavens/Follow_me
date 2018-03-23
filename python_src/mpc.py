import cvxopt as cvx
import numpy as np
import math


#constant speed value for now

#return jacobians for ackerman steering kinematics given a reference center
def ackerman_model(state_center, input_center, delta_time):
	

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

	def __init__(dynamics, state_0, u_min, u_max, u_cost, horizon,track_weighting):
		
		self.dynamics = dynamics
		self.u_min = u_min
		self.u_max = u_max
		self.u_cost = u_cost
		self.track_weighting = track_weighting
		self.horizon = horizon

		P, H = dyn.pred_mat(state_0)
        self.P = P


    def construct_pred_mat(self,ref_traj, ref_inputs):
    	a_dim = len(ref_traj[0])
    	b_dim = len(ref_inputs[0])
    	#initialize matricies
    	P = np.zeros((a_dim*horizon,a_dim))
    	H = np.zeros((a_dim*horizon,b_dim*horizon))

    	As = np.zeros((a_dim,a_dim*horizon))
    	Bs = np.zeros((a_dim,b_dim*horizon))
    	#last_A = np.zeros((a_dim,a_dim))
    	for i in range(horizon):
    		index_a = i*a_dim
    		index_b = i*b_dim

    		As[:,index_a:index_a+a_dim], Bs[:,index_b:index_b+index_b] = self.dynamics(ref_traj[0],ref_inputs[0])
    		
    		if "last_A" not in locals():
    			As[:,index_a:index_a+a_dim] = A
    			last_A = A
    		else:
    		As[:,index_a:index_a+a_dim] = np.dot(A,last_A)
    	for 
	def compute_linearized_dyn(self, ref_traj):
		P, H = self.pred_mat(ref_traj)
		self.P = P
		self.H = H

	def optimize_u(self, ref_traj):


