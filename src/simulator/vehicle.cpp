#include "vehicle.h"
#include<math.h>
#include<cmath>
#include<iostream>

using namespace std;


vehicle::vehicle(const double x_, const double y_, const double theta_, const double speed_,const double wheel_base_l)
{
	x = x_;
	y = y_;
	theta = theta_;
	speed = speed_;
	wb_L = wheel_base_l;//m
	max_speed = 10;//m/s
	max_steer = M_PI/2;//rad
	max_throttle = 4.0;//m/s^2

}

double vehicle::usec_to_sec(const unsigned long t_usec)
{
	return (double)(t_usec)/1000000.0;	
}

void vehicle::update(double steer_u, double throttle_u, const unsigned long dt_usec)
{
	double dt = usec_to_sec(dt_usec);
	if(abs(throttle_u) > max_throttle)
		throttle_u = max_throttle*throttle_u/abs(throttle_u);
	if(abs(steer_u) > max_steer)
		steer_u = max_steer*steer_u/abs(steer_u);
	
	theta += speed/wb_L*steer_u*dt;
	theta = pi_2_pi(theta);
	speed += throttle_u*dt;


	cout<<theta<<","<<speed<<endl;
	if(abs(speed) > max_speed)
		speed = max_speed*speed/abs(speed);
	x += speed*cos(theta)*dt;
	y += speed*sin(theta)*dt;
}

Eigen::VectorXd vehicle::get_state()
{
	Eigen::VectorXd ret_state(4);
	ret_state << x,y,theta,speed;
	return ret_state;
}

double pi_2_pi(const double theta)
{
	double ret_theta = theta;
	while(ret_theta > M_PI)
		ret_theta -= 2.0*M_PI;
	while(ret_theta < M_PI)
		ret_theta += 2.0*M_PI;
	return ret_theta;
}

