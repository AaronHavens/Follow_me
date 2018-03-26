#import cvxopt as cvx
import numpy as np
import math
import time

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

    def __init__(self,dynamics, state_0,input_0, u_min, u_max, u_cost, horizon,track_weighting):
        
        self.dynamics = dynamics
        self.u_min = u_min
        self.u_max = u_max
        self.u_cost = u_cost
        self.track_weighting = track_weighting
        self.horizon = horizon
        self.state_dim = np.size(state_0)
        self.u_dim = np.size(input_0)
        self.ref_traj = None
        self.ref_input = None
        self.R = self.input_weight_mat(u_cost)
        self.Q = self.track_weight_mat(track_weighting)
        #P, H = self.construct_pred_mat(state_0)
        #self.P = P
        #self.H = H


    def construct_pred_mat(self,ref_traj, ref_inputs):
        a_dim = self.state_dim #temp state space discription
        b_dim = self.u_dim
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

        j_index = horizon
        for i in range(horizon):
            i_index = 1
            H[i*a_dim:i*a_dim+a_dim,i*b_dim:i*b_dim+b_dim] = Bs[i]
            for j in range(i):
                H[i*a_dim:i*a_dim+a_dim,j*b_dim:j*b_dim+b_dim] = np.dot(alpha[i_index*a_dim:i_index*a_dim+a_dim,j_index*a_dim:j_index*a_dim+a_dim],Bs[j])
                i_index += 1
            
            j_index -= 1

        return P, H

    def optimize_unconstrained(self, ref_traj,ref_inputs,state):
        P,H = self.construct_pred_mat(ref_traj,ref_inputs)
        x_tild = self.state_error(ref_traj[0],state)
        x_r_tild = self.state_error_ref(ref_traj)
        u_r_tild = self.input_error_ref(ref_inputs)
        print(np.shape(P),np.shape(H),np.shape(x_tild),np.shape(x_r_tild),np.shape(u_r_tild),np.shape(self.Q),np.shape(self.R))
        u_tild_comp1 = np.linalg.inv(np.dot(np.dot(np.transpose(H),self.Q),H)+self.R)
        u_tild_comp2 = np.dot(np.dot(np.transpose(H),self.Q),(x_r_tild - np.dot(P,x_tild))) + np.dot(self.R,u_r_tild)

        u_tild = np.dot(u_tild_comp1,u_tild_comp2)
        self.ref_inputs = u_tild
        return u_tild


    def compute_linearized_dyn(self, ref_traj,ref_input):
        P, H = self.construct_pred_mat(ref_traj,ref_inputs)
        self.P = P
        self.H = H

    def state_error_ref(self,ref_traj):
        h = self.horizon
        dim = self.state_dim
        ref_error = np.zeros((h*dim,1))
        for i in range(h):
            ref_error[i*dim:i*dim+dim,0] = ref_traj[(i+1)*dim:(i+1)*dim+dim,0] - ref_traj[0:dim,0]
        return ref_error

    def input_error_ref(self,ref_inputs):
        h = self.horizon
        dim = self.u_dim
        input_error = np.zeros((h*dim,1))
        for i in range(h):
            input_error[i*dim:i*dim+dim,0] = ref_inputs[(i+1)*dim:(i+1)*dim+dim,0] - ref_inputs[0:dim,0]
        return input_error

    def state_error(self,state,state_ref):
        return state - state_ref

    def track_weight_mat(self,weight):
        track_weighting = np.zeros((self.state_dim*self.horizon,self.state_dim*self.horizon))
        for i in range(self.horizon*self.state_dim):
            track_weighting[i,i] = weight
        return track_weighting

    def input_weight_mat(self,weight):
        input_weighting = np.zeros((self.u_dim*self.horizon,self.u_dim*self.horizon))
        for i in range(self.horizon*self.u_dim):
            input_weighting[i,i] = weight
        return input_weighting

def construct_trajectory(horizon,state_0,vf,delta_t):
    dim = np.size(state_0)
    ref_traj = np.zeros((horizon*dim,1))
    ref_inputs = np.zeros((horizon,1))
    ref_traj[0:3,0] = state_0[:,0]
    for i in range(horizon):
        ref_inputs[i,0] = 0.0 + np.random.uniform(-0.05, 0.05)
    for i in range(horizon):
        x = state_0[0,0]
        y = state_0[1,0]
        theta = state_0[2,0]
        state = np.zeros((np.size(state_0),1))
        state[0,0] = x + vf*math.cos(theta+ref_inputs[i])*delta_t
        state[1,0] = y + vf*math.sin(theta+ref_inputs[i])*delta_t
        state[2,0] = theta + vf*delta_t/1.0*math.sin(ref_inputs[i])
        ref_traj[i*dim:i*dim+dim,:] = state
        state_0 = state
        
    return ref_traj, ref_inputs

state = np.array([[5.0],[1.0],[0.0]]).reshape((3,1))
input_sample = np.array([0.0])
#[Px'-]
horizon = 10
u_min = -2.0
u_max = 2.0
u_cost = 1.0
track_weighting = 1.0



controller = MPC_controller(ackerman_model,state,input_sample, u_min, u_max, u_cost, horizon,track_weighting)
ref_traj, ref_inputs = construct_trajectory(horizon+1,state,5.0,0.1)
start = time.time()
u=controller.optimize_unconstrained(ref_traj, ref_inputs,state)
end = time.time()
print(u)
print(end - start)


