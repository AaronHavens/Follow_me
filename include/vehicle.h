//author: aaron havens
#include<eigen3/Eigen/Dense>


class vehicle
{


	public:
		vehicle(const double x, const double y,
		const double theta, const double speed,
		const double wheel_base_l);

		void update(double steer_u, double throttle_u,const unsigned long dt_usec);
		Eigen::VectorXd get_state();
	private:
		double x,y,theta,speed,wb_L;
		
		double max_speed, max_steer, max_throttle;

		double usec_to_sec(const unsigned long t_usec);



};

double pi_2_pi(const double theta);
