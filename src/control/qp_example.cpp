#include <qpOASES.hpp>
using namespace qpOASES;
int main()
{

	//setup 1st QP
	real_t H[2*2] = {1.0,0.0,0.0,0.5};
	real_t A[1*2] = {1.0,1.0};
	real_t g[2] = {1.5,1.0};
	//constraints
	real_t lb[2] = {0.5,-2.0};
	real_t ub[2] = {5.0,2.0};
	real_t lbA[1] = {-1.0};
	real_t ubA[1] = {2.0};
	//setup 2nd QP
	real_t g_new[2] = {1.0,1.5};
	real_t lb_new[2] = {0.0,-1.0};
	real_t ub_new[2] = {5.0,-0.5};
	real_t lbA_new[1] = {-2.0};
	real_t ubA_new[1] = {1.0};

	//setup Qp problem object
	QProblem example(2,1);
	
	//and then solve 1st qp
	int_t nWSR = 10;
	example.init(H,g,A,lb,ub,lbA,ubA,nWSR);

	//solve 2nd QP
	nWSR = 10;
	example.hotstart(g_new,lb_new,ub_new,lbA_new,ubA_new,nWSR);

	//print solutions
	real_t xOpt[2];
	example.getPrimalSolution(xOpt);
	printf("\n xOpt = [%e,%e]; objVal = %e \n\n",
				xOpt[0],xOpt[1],example.getObjVal() );

	return 0;
}
