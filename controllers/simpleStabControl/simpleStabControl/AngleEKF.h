

#ifndef ANGLE_EKF_H
#define ANGLE_EKF_H
#include "EKF.h"
#include "EKFStructs.h"
#include <Eigen/Core>
#include <Eigen/Dense>
#include "Common.h"
#include "MMath.h"

class AngleEKF
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	AngleEKF(void);
	~AngleEKF(void);




	void update(const float angleX,
		const float angleY);	

	int correctionStep(std::vector<AngleMeasurement> z_k);



private: 
	static const int num_dimensions;
	static const float variance;
	static const float beta;
	static const float gamma;
 unsigned int numStates; // числоо состояние фильтра Калмана
	void timeUpdate(int);
	const float scale(const float x) ;

	Eigen::Vector2f xhat_k;
	Eigen::Matrix2f A_k; 
	Eigen::Matrix2f P_k; 
	Eigen::Vector2f xhat_k_bar;
	Eigen::Matrix2f Q_k; 
	Eigen::Matrix2f P_k_bar; 
	Eigen::Vector2f betas; 
	Eigen::Vector2f gammas;

	int frameCounter;
	


	 void incorporateMeasurement(AngleMeasurement z,
		Eigen::Matrix2f &H_k,
		Eigen::Matrix2f &R_k,
		Eigen::Vector2f &V_k);

public:

	 float getAngleX() const;
	 float getAngleY() const;
	
	AngleEKF & operator=(const AngleEKF & other){
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
};

#endif // ANGLE_EKF_H


