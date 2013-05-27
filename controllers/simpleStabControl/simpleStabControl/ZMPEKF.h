#ifndef ZMPEKF_H
#define ZMPEKF_H
#include "EKFStructs.h"
#include "EKF.h"
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/vector.hpp>


class ZmpEKF 
{
public:
    ZmpEKF();
     ~ZmpEKF();

	
	ZmpEKF & operator=(const ZmpEKF & other){
		if(this != &other){
			xhat_k     = other.xhat_k;
			xhat_k_bar = other.xhat_k_bar;
			Q_k        = other.Q_k;
			A_k        = other.A_k;
			P_k        = other.P_k;
			P_k_bar    = other.P_k_bar;
			betas      = other.betas;
			gammas     = other.gammas;
		}
		return *this;
	}


	static const int dimension = 2;
	static const int mSize = 2;

	const float get_zmp_x() const { return xhat_k(0); }
	const float get_zmp_y() const { return xhat_k(1); }

	typedef boost::numeric::ublas::vector<float, boost::numeric::ublas::
		bounded_array<float,dimension> >
		StateVector;
	// A vector with the length of the measurement dimensions
	typedef boost::numeric::ublas::vector<float, boost::numeric::ublas::
		bounded_array<float,mSize> >
		MeasurementVector;

	// A square matrix with state dimension number of rows and cols
	typedef boost::numeric::ublas::matrix<float,
		boost::numeric::ublas::row_major,
		boost::numeric::ublas::
		bounded_array<float, dimension*
		dimension> >
		StateMatrix;

	// A square matrix with measurement dimension number of rows and cols
	typedef boost::numeric::ublas::matrix<float,
		boost::numeric::ublas::row_major,
		boost::numeric::ublas::
		bounded_array<float, mSize*
		mSize> >
		MeasurementMatrix;

	// A matrix that is of size measurement * states
	typedef boost::numeric::ublas::matrix<float,
		boost::numeric::ublas::row_major,
		boost::numeric::ublas::
		bounded_array<float, mSize*
		dimension> >
		StateMeasurementMatrix;


private: // Constants
    static const float beta;
    static const float gamma;
	StateVector xhat_k; // Estimate Vector
	StateVector xhat_k_bar; // A priori Estimate Vector
	StateMatrix Q_k; // Input noise covariance matrix
	StateMatrix A_k; // Update measurement Jacobian
	StateMatrix P_k; // Uncertainty Matrix
	StateMatrix P_k_bar; // A priori uncertainty Matrix
	const boost::numeric::ublas::identity_matrix<float> dimensionIdentity;
	const unsigned int numStates; // number of states in the kalman filter
	const unsigned int measurementSize; // dimension of the observation (z_k)
	 StateVector betas; // constant uncertainty increase
	StateVector gammas; // scaled uncertainty increase
	int frameCounter;
};



#endif //ZMPEKF_H