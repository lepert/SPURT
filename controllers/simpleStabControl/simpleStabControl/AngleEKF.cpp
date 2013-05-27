

#include "AngleEKF.h"
 const float DONT_PROCESS_KEY = -1337.0f;
 const int ANGLE_NUM_DIMENSIONS = 2;
const int AngleEKF::num_dimensions = ANGLE_NUM_DIMENSIONS;
const float AngleEKF::beta = 3.0f;
const float AngleEKF::gamma = 2.0f;
const float AngleEKF::variance  = 0.22f;
//const float AccEKF::variance  = 100.00f;

AngleEKF::AngleEKF(void)
	:frameCounter(0) 
{

	Q_k=Eigen::Matrix2f::Zero();
	A_k =Eigen::Matrix2f::Zero();
	P_k =Eigen::Matrix2f::Zero();
	P_k_bar =Eigen::Matrix2f::Zero();

	xhat_k_bar=Eigen::Vector2f::Zero();
	xhat_k=Eigen::Vector2f::Zero();

	betas<<beta,beta;
	gammas<<gamma,gamma;


	numStates=2;

	A_k(0,0) = 1.0;
	A_k(1,1) = 1.0;


	P_k(0,0) = -GRAVITY_mss;
	P_k(1,1) = -GRAVITY_mss;

}





AngleEKF::~AngleEKF(void)


{
}

void AngleEKF::update( const float angleX, const float angleY )
{
	
	
	timeUpdate(0); 
	AngleMeasurement m = { angleX, angleY };
	std::vector<AngleMeasurement> z(1,m);

	correctionStep(z);

}


void AngleEKF::incorporateMeasurement(AngleMeasurement z, Eigen::Matrix2f &H_k, Eigen::Matrix2f &R_k, Eigen::Vector2f &V_k )
{

	static Eigen::Vector2f last_measurement(Eigen::Vector2f::Zero());
	  Eigen::Vector2f z_x(Eigen::Vector2f::Zero());

	  z_x(0) = z.angleX;
	  z_x(1) = z.angleY;



	   V_k = z_x - xhat_k; 

	   //  Jacobian 
	   H_k(0,0) = 1.0f;
	   H_k(1,1) = 1.0f;

	   Eigen::Vector2f deltaS = z_x - last_measurement;


	   R_k(0,0) = scale(std::abs(V_k(0)));
	   R_k(1,1) = scale(std::abs(V_k(1)));

	       last_measurement = z_x;

		  

}




int AngleEKF::correctionStep(std::vector<AngleMeasurement> z_k)
{


	// Kalman gain matrix
	Eigen::Matrix2f K_k =Eigen::Matrix2f::Zero();

	// Observation jacobian
	Eigen::Matrix2f  H_k =Eigen::Matrix2f::Zero();
	
	// Assumed error in measurment sensors
	Eigen::Matrix2f R_k = Eigen::Matrix2f::Zero();

	// Measurement invariance
	Eigen::Vector2f v_k=	Eigen::Vector2f::Zero();





	// Incorporate all correction observations

	for (std::vector<AngleMeasurement>::iterator i=z_k.begin();i!=z_k.end();i++)

	{


		incorporateMeasurement((*i), H_k, R_k, v_k);

		

		if (R_k(0,0) == DONT_PROCESS_KEY) {
	
		
			return 0;

		}


		// Calculate the Kalman gain matrix

		

		const Eigen::Matrix2f pTimesHTrans = (P_k_bar* H_k);

			
			const Eigen::Matrix2f pinv=(  R_k+(H_k* pTimesHTrans) ).inverse();
			
			K_k = pTimesHTrans*pinv;

			
			// Use the Kalman gain matrix to determine the next estimate

			

			xhat_k_bar = xhat_k_bar + (K_k * v_k);

			
			Eigen::Matrix2f  nd=	Eigen::Matrix2f::Identity();


			// Update associate uncertainty

			P_k_bar = (nd - (K_k*H_k))* P_k_bar;



	}




	xhat_k = xhat_k_bar;


	P_k = P_k_bar;



	return 0;
}

void AngleEKF::timeUpdate( int u_k)
{
	++frameCounter;

	Eigen::Vector2f deltas=Eigen::Vector2f::Zero();
	
	xhat_k_bar = xhat_k + deltas;

	
	for(unsigned int i = 0; i < 2; i++) {
		Q_k(i,i) = beta ;
	}
Eigen::Matrix2f A_kt=(A_k).transpose();
	
	Eigen::Matrix2f newP = (P_k* A_kt);
	P_k_bar = (A_k* newP) + Q_k;




}

float AngleEKF::getAngleX() const 
{
return xhat_k(0); 
}

float AngleEKF::getAngleY() const 
{
	return xhat_k(1); 
}

const float AngleEKF::scale( const float x )
{

   return 100.0f * std::pow(x, 5.0f) + 580.4f;
    

}


