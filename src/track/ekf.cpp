#include"ekf.h"
#include<iostream>
#include<math.h>

ekf::ekf(const VectorXd prior_state,const unsigned long t_usec)
{
	state_size = prior_state.size();
	state = prior_state;
	cov = MatrixXd::Ones(state_size,state_size)*0.1;
	last_update = t_usec;
	F = MatrixXd::Zero(state_size,state_size);
	H = MatrixXd::Zero(2,state_size);
}	
	
void ekf::propogate_model(const unsigned long t_usec)
{

	double dt = (double)(t_usec - last_update)/1000000.0;
	set_last_update(t_usec);
	set_jacobian_F(dt);
	state = f_x(dt);
	cov = F.transpose()*cov*F + proccess_noise_Q(dt);
}


void ekf::measurement_update(const VectorXd z,const unsigned long t_usec)
{
	//convert from usec to sec
	double dt = (double)(t_usec - last_update)/1000000.0;	
	//propogate state forward for prediction
	propogate_model(t_usec);	
	
	set_jacobian_H();

	VectorXd y_innovation = z - h_x();
	MatrixXd H_T = H.transpose();
	MatrixXd S = H*cov*H_T + proccess_noise_Q(dt).block(0,0,2,2);
	
	MatrixXd K = cov*H_T*S.inverse();

	state += K*y_innovation;
	MatrixXd I = MatrixXd::Identity(state_size,state_size);
	
	cov = (I-K*H)*cov;	
}

void ekf::set_jacobian_F(const double dt)
{
	double x = state(0);
	double y = state(1);
	double theta = state(2);
	double v = state(3);
	F << 	1.0,0,-sin(theta)*v*dt, cos(theta)*dt,
		0,1.0,cos(theta)*v*dt, sin(theta)*dt,
		0,0,1.0,0,
		0,0,0,1.0;
}
void ekf::set_jacobian_H()
{

	H << 	1.0,0,0,0,
		0,1.0,0,0;

}

MatrixXd ekf::proccess_noise_Q(const double dt)
{
	double dt_2 = dt*dt;
	double dt_3 = dt_2*dt;
	double dt_4 = dt_3*dt;
	double dt_4_4 = dt_4/4.0;
	double dt_3_2 = dt_3/2.0;

	MatrixXd Q(state_size,state_size);

	Q <<	dt_4_4,0,dt_3_2,0,
		0,dt_4_4,0,dt_3_2,
		dt_3_2,0,dt_2,0,
		0,dt_3_2,0,dt_2;
	return Q;
}

VectorXd ekf::f_x(const double dt)
{
	double theta = state(2);
	double v = state(3);
	VectorXd state_hat = state;
	state_hat(0) += v*cos(theta)*dt;
	state_hat(1) += v*sin(theta)*dt;
	return state_hat;
}

VectorXd ekf::h_x()
{
	VectorXd z_hat(2);
	z_hat = state.segment(0,2);
	return z_hat;
}
