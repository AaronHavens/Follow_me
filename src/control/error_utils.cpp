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

	x_2 = vl(0);
	y_2 = vl(1);

	VectorXd v1(2);
	v1 << v_1 * cos(theta_1),v_1*sin(theta_1);
	double a = -v1(1);
	double b = v1(0); 
	double c = -a*x_1 - b*y_1;
	double cte = abs(a*x_2+b*y_2 + c)/sqrt(a*a + b*b);

	
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
	v2 << x_2 - x_1, y_2 - y_1;
	double norm_v1 = v1(0)*v1(0) + v1(1)*v1(1);
	VectorXd proj = ((v2*v1)/norm_v1)*v1;
	double dx = proj(0)-x_1;
	double dy = proj(1)-y_1;

	double ote = sqrt(dx*dx+dy*dy);
	return ote;
}
