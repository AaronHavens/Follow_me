#include<eigen3/Eigen/Dense>

using namespace Eigen;
using namespace std;


class ekf
{
	private:
		VectorXd state;
		MatrixXd cov;
		unsigned int state_size;
		MatrixXd F;
		MatrixXd H;
		unsigned long last_update;

		void set_last_update(const unsigned long t_usec){last_update = t_usec;};

		void set_jacobian_F(const double dt);
		void set_jacobian_H();
		MatrixXd proccess_noise_Q(const double dt);
		//MatrixXd observer_noise_R(double dt);
		VectorXd f_x(const double dt);
		VectorXd h_x();

	public:
		ekf(const VectorXd prior,const unsigned long t_usec);

		unsigned long get_last_update(){return last_update;};	
		VectorXd get_state(){return state;};
		MatrixXd get_cov(){return cov;};
		void measurement_update(const VectorXd z,const unsigned long t_usec);
		void propogate_model(const unsigned long t_usec);
};
