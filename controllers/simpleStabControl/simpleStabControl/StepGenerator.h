#ifndef STEP_GENERATOR_H
#define STEP_GENERATOR_H
#include <iostream>
#include <list>
#include "Observer.h"
#include "GaitDefaults.h"
#include "boost/tuple/tuple.hpp"

#include "Step.h"
#include "ZMPEKF.h"
#include "SensorAngles.h"
#include "WalkingLeg.h"
#include "Gait.h"
#include "Kinematics.h"
#include "CoordFrame.h"
#include "ZMPEKF.h"
#include "WalkingLeg.h"
#include <Eigen/Core>
#include "CoordFrame.h"
#include <Eigen/Dense>
#include "WalkingArm.h"



typedef boost::tuple<const std::list<float>*,
	const std::list<float>*> zmp_xy_tuple;
typedef boost::tuple<LegJointStiffTuple,
	LegJointStiffTuple> WalkLegsTuple;
typedef boost::tuple<ArmJointStiffTuple,
	ArmJointStiffTuple> WalkArmsTuple;

static unsigned int MIN_NUM_ENQUEUED_STEPS = 3; 

using namespace GaitDefaults;
class StepGenerator
{

public:

	 EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	StepGenerator(std::tr1::shared_ptr<GeneralSensors> s);
	~StepGenerator(void);
	WalkArmsTuple tick_arms();
	 WalkLegsTuple tickLegs();
	 void resetOdometry(const float initX, const float initY);
	bool done;
	void takeSteps(const float _x, const float _y, const float _theta,
		const int _numSteps);
	void updateOdometry(const vector<float> &deltaOdo);
	/**когда робот начинает движение 
	с 0 нужно освобождать все блоки 
	шагов и значений змп*/
	void resetSteps(const bool startLeft);
	void resetQueues();
	void generateStep(float _x,float _y,
		float _theta);
	void clearFutureSteps();
	/**настройка локоматива на
	движение с определенной ноги*/
	void tick_controller();
	void setSpeed(const float _x, const float _y,
		const float _theta) ;
	 void swapSupportLegs();
	zmp_xy_tuple generate_zmp_ref();

	static const Eigen::Matrix3f get_fprime_f(const std::tr1::shared_ptr<Step> step);
	static const Eigen::Matrix3f  get_f_fprime(const std::tr1::shared_ptr<Step> step);



private:
	    WalkingArm leftArm, rightArm;
	int counter;
	 SensorAngles sensorAngles;
	// std::list<std::tr1::shared_ptr<Step>> futureSteps;

	 /**функция определения маховой ноги
	 если боковое движение или поворот уходит
	 налево, то будет выбрано левое наравление*/

	 const bool decideStartLeft(const float lateralVelocity,
		 const float radialVelocity);
	std::tr1::shared_ptr<Step> lastQueuedStep;
		Observer  *mController_x, *mController_y;
		Eigen::Matrix3d mSi_Transform;
		Eigen::Vector3f mLast_zmp_end_s;
		Gait gait;
		bool nextStepIsLeft;
		std::list<std::tr1::shared_ptr<Step> > futureSteps; //не обрабоанные змп значения
		float x;
		float y;
		float theta;
		 void fillZMP(const std::tr1::shared_ptr<Step> newStep );
		  void fillZMPRegular(const std::tr1::shared_ptr<Step> newStep );
	
		std::list<float> zmp_ref_x, zmp_ref_y;
		 std::list<std::tr1::shared_ptr<Step> > currentZMPDSteps;
		   Eigen::Vector3f last_zmp_end_s,com_i,com_f;
		    Eigen::Matrix3f si_Transform;
			   static const Eigen::Matrix3f get_sprime_s(const std::tr1::shared_ptr<Step> step);
			   static const Eigen::Matrix3f get_s_sprime(const std::tr1::shared_ptr<Step> step);
			     void fillZMPEnd(const std::tr1::shared_ptr<Step> newStep );
				  Eigen::Vector3f est_zmp_i;
				   ZmpEKF zmp_filter;
				   float scaleSensors(const float sensorZMP, const float perfectZMP);
		Observer *controller_x, *controller_y;
		   WalkingLeg leftLeg, rightLeg;
		    Kinematics::SupportFoot supportFoot;


		  
		  std::tr1::shared_ptr<Step> lastStep_s;
		    std::tr1::shared_ptr<Step> supportStep_s;
		    std::tr1::shared_ptr<Step> swingingStep_s;
		    std::tr1::shared_ptr<Step> supportStep_f;
		    std::tr1::shared_ptr<Step> swingingStep_f;
		    std::tr1::shared_ptr<Step> swingingStepSource_f;


			
			Eigen::Matrix3f if_Transform;
			Eigen::Matrix3f fc_Transform;
			Eigen::Matrix3f cc_Transform; 

};

#endif// STEP_GENERATOR_H