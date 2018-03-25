#import cvxopt as cvx
import numpy as np
import math


#constant speed value for now

#return jacobians for ackerman steering kinematics given a reference center
def ackerman_model(state_center, input_center, delta_time,vf=5.0):
    

    delta_ref  = input_center[0]
    theta_ref = state_center[0]

    A = np.array([[1,0,-vf*delta_time*math.sin(theta_ref+delta_ref)],
                [0,1,vf*delta_time*math.cos(theta_ref + delta_ref)],
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
        #self.P = P
        #self.H = H


    def construct_pred_mat(self,ref_traj, ref_inputs):
        a_dim = 3 #temp state space discription
        b_dim = len(ref_inputs[0])
        #initialize matricies
        P = np.zeros((a_dim*horizon,a_dim))
        H = np.zeros((a_dim*horizon,b_dim*horizon))

        As = []
        Bs = []
        #last_A = np.zeros((a_dim,a_dim))
        for i in range(horizon+1):
            index_a = i*a_dim
            index_b = i*b_dim

            A, B = self.dynamics(ref_traj[i],ref_inputs[i],.1)
            As.append(A)
            Bs.append(B)
            
        alpha = np.zeros((a_dim*(horizon+1),a_dim*(horizon+1)))
        for i in range(horizon+1):
            A_sum = np.zeros((a_dim,a_dim))
            items_sumed = []
            for j in range(i,horizon+1):
                if A_sum.all() == 0.0:
                    A_sum = As[j]
                else:
                    A_sum = np.dot(As[j],A_sum)
                l=horizon-j
                items_sumed.append(j)
                alpha[i*a_dim:i*a_dim+a_dim,l*a_dim:l*a_dim+a_dim] = A_sum
        i_index = 0
        for i in reversed(range(horizon)):
            P[i_index*a_dim:i_index*a_dim+a_dim,:] = alpha[0:a_dim,i*a_dim:i*a_dim+a_dim]
            i_index += 1

        i_index = 0 
        for i in range(horizon):
            j_index = horizon-1
            for j in range(i+1):
                H[i*a_dim:i*a_dim+a_dim,j*b_dim:j*b_dim+b_dim] = np.dot(alpha[i_index*a_dim:i_index*a_dim+a_dim,j_index*a_dim:j_index*a_dim+a_dim],Bs[j])
                
                j_index -= 1
            if i == j: H[i*a_dim:i*a_dim+a_dim,j*b_dim:j*b_dim+b_dim] = Bs[j]
            i_index += 1

        return P, H

    def compute_linearized_dyn(self, ref_traj):
        P, H = self.pred_mat(ref_traj)
        self.P = P
        self.H = H

def construct_trajectory(horizon):
    ref_traj = np.zeros((horizon,1))
    ref_inputs = np.zeros((horizon,1))
    for i in range(horizon):
        ref_traj[i,0] = 0.0 + np.random.uniform(-0.05,0.05)
        ref_inputs[i,0] = 0.0 + np.random.uniform(-0.05, 0.05)
    return ref_traj, ref_inputs

state = np.array([5.0,1.0,0.0])

horizon = 2
u_min = -2.0
u_max = 2.0
u_cost = 1.0
track_weighting = 0



controller = MPC_controller(ackerman_model,state, u_min, u_max, u_cost, horizon,track_weighting)
ref_traj, ref_inputs = construct_trajectory(horizon+1)

P,H=controller.construct_pred_mat(ref_traj, ref_inputs)
print(P)
print(H)
