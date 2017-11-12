#include<eigen3/Eigen/Dense>

using namespace Eigen;



double cross_track_e(VectorXd vf, VectorXd vl);

double on_track_e(VectorXd vf, VectorXd vl);

VectorXd all_track_e(VectorXd vehicle_state, VectorXd ref_state);

VectorXd to_global_frame(VectorXd vehicle_state);
