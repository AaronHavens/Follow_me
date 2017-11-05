#include<eigen3/Eigen/Dense>
#include<iostream>

using namespace Eigen;
using namespace std;

int main ()
{
	MatrixXd M(3,3);
	M << 	1,1,1,
		2,2,2,
		3,3,3;
	cout<<M<<endl;
}
