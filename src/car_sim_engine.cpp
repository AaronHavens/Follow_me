#include<eigen3/Eigen/Dense>
#include<iostream>
#include "ekf.h"

using namespace Eigen;
using namespace std;

int main ()
{
	MatrixXd M(3,3);
	M << 	1,1,1,
		2,2,2,
		3,3,3;
	cout<<M<<endl;
	VectorXd prior(4);
	prior << 0,0,0,1;
	unsigned long time_usec = 50012030;

	ekf filter_test(prior,time_usec);
	cout<<filter_test.get_state()<<endl;	
	cout<<filter_test.get_last_update()<<endl;
	filter_test.propogate_model(filter_test.get_last_update() + 1000000);
	cout<<filter_test.get_state()<<endl;
	cout<<filter_test.get_cov()<<endl;
	VectorXd z(2);
	z << 2.5,0;
	filter_test.measurement_update(z,filter_test.get_last_update()+1000000);
	cout <<filter_test.get_state()<<endl;
	cout <<filter_test.get_cov()<<endl;
	z << 3.2,0.1;
	filter_test.measurement_update(z,filter_test.get_last_update()+1000000);
	cout <<filter_test.get_state()<<endl;
	cout <<filter_test.get_cov()<<endl;
}
