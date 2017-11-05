#include<eigen3/Eigen/Dense>
#include<iostream>
#include "ekf.h"
#include "vehicle.h"

using namespace Eigen;
using namespace std;

int main ()
{
	vehicle car(0.0,0.0,0.0,0.0,2.0);
	car.update(0.0,2.0,1000000);	
	cout << car.get_state() <<endl;	
}
