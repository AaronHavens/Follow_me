#include<eigen3/Eigen/Dense>
#include<iostream>
#include"ekf.h"
#include"vehicle.h"
#include"error_utils.hpp"
#include<math.h>

using namespace Eigen;
using namespace std;

int main ()
{
	vehicle car(0.0,0.0,0.0,0.0,2.0);
	for(unsigned int i = 0; i < 100; i++){
		car.update(0.5,1.0,1000000);
		cout<<car.get_state()<<endl;
		cout<<"---------------"<<endl;
	}
	VectorXd vf(4);
	VectorXd track(4);
	vf << 1,0,M_PI/4,1;
	track<<0,0,M_PI/4,1;
	double error = cross_track_e(vf,track);
	cout << error <<endl;
}
