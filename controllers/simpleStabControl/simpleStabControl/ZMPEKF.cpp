#include "ZMPEKF.h"
#include "Kinematics.h"
const float ZmpEKF::beta = 0.1f;
const float ZmpEKF::gamma = 0.5f;
//const float ZmpEKF::variance  = 100.00f;


	

ZmpEKF::ZmpEKF()
	: xhat_k(dimension), xhat_k_bar(dimension),
	Q_k(dimension,dimension), A_k(dimension,dimension),
	P_k(dimension,dimension), P_k_bar(dimension,dimension),
	dimensionIdentity(dimension), numStates(dimension),
	measurementSize(mSize), betas(dimension), gammas(dimension),
	frameCounter(0)
	{

	A_k(0,0) = 1.0;
	A_k(1,1) = 1.0;

	
	xhat_k(0) = 0.0f;
	xhat_k(1) = 0.0f;

	
	P_k(0,0) = Kinematics::HIP_OFFSET_Y;
	P_k(1,1) = Kinematics::HIP_OFFSET_Y;

}

ZmpEKF::~ZmpEKF()
{

}

