#include"error_utils.hpp"
#include<math.h>

using namespace std;




double cross_track_e(VectorXd vf, VectorXd vl)
{	
	
	double x_1,y_1,theta_1,v_1;
	double x_2,y_2;
	

	x_1 = vf(0);
	y_1 = vf(1);
	theta_1 = vf(2);
	v_1 = vf(3);
	MatrixXd T(3,3);
	x_2 = vl(0);
	y_2 = vl(1);

	VectorXd v1(2);
	v1 << v_1 * cos(theta_1),v_1*sin(theta_1);
	double a = -v1(1);
	double b = v1(0); 
	double c = -a*x_1 - b*y_1;
	double cte = abs(a*x_2+b*y_2 + c)/sqrt(a*a + b*b);

	return cte;	
}

double on_track_e(VectorXd vf, VectorXd vl)
{
	double x_1,y_1,theta_1,v_1;
	double x_2,y_2;
	

	x_1 = vf(0);
	y_1 = vf(1);
	theta_1 = vf(2);
	v_1 = vf(3);

	x_2 = vl(0);
	y_2 = vl(1);

	VectorXd v1(2);
	VectorXd v2(2);
	v1 << v_1 * cos(theta_1),v_1*sin(theta_1);
	v2 << x_1 - x_2, y_1 - y_2;
	double norm_v1 = v1(0)*v1(0) + v1(1)*v1(1);
	VectorXd proj = (v2.dot(v1)/norm_v1)*v1;
	double dx_2 = proj(0)*proj(0);
	double dy_2 = proj(1)*proj(1);

	double ote = sqrt(dx_2+dy_2);
	return ote;
}

VectorXd all_track_e(VectorXd vehicle_state, VectorXd ref_state)
{
	double thetac = vehicle_state(2);

	VectorXd diff = ref_state.segment(0,3) - vehicle_state.segment(0,3);

	MatrixXd T(3,3);

	T <<cos(thetac), sin(thetac), 0,
			-sin(thetac), cos(thetac),0,
			0, 0, 1;

	VectorXd track_errors = T*diff;
	
	return track_errors; //[x_e,y_e,theta_e]^T from vehicle reference.
		
}

VectorXd to_global_frame(VectorXd vehicle_state)
{
	double theta_car = vehicle_state(2);

	MatrixXd T(3,2);
	T <<cos(theta_car), 0,
			sin(theta_car), 0,
			0, 1;

	VectorXd global_state = T * vehicle_state.segment(2,2);
	return global_state;
}
