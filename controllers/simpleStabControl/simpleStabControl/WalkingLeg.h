#ifndef WALKING_LEG_H 
#define WALKING_LEG_H

#include <cstdio>
#include <string>
#include <vector>
#include "Gait.h"
#include "Step.h"
#include <boost/tuple/tuple.hpp>
#include "Kinematics.h"
#include <Eigen/Core>
#include <Eigen/Dense>
#include "SensorAngles.h"
#include "InverseKinematics.h"
#include "WalkingConstants.h"

typedef boost::tuple<std::vector<float>,
	std::vector<float> > LegJointStiffTuple;

/*
enum SupportMode{
	SUPPORTING=0,
	SWINGING,
	DOUBLE_SUPPORT,
	PERSISTENT_DOUBLE_SUPPORT
};*/

enum JointStiffIndex {
	JOINT_INDEX,
	STIFF_INDEX
};

class WalkingLeg  {
public:
	WalkingLeg(   const Gait * _gait,const SensorAngles * _sensorAngles,
		Kinematics::ChainID id);
	~WalkingLeg();
	    bool isSwitchingSupportMode(){return firstFrame();}

		bool stateIsDoubleSupport(){
			return state == DOUBLE_SUPPORT ||
				state == PERSISTENT_DOUBLE_SUPPORT;
		};
		void setSteps(std::tr1::shared_ptr<Step> _swing_src,
			std::tr1::shared_ptr<Step> _swing_dest,
			std::tr1::shared_ptr<Step> _suppoting);
		 const float getFootRotation();

		 LegJointStiffTuple tick(std::tr1::shared_ptr<Step> step,
			 std::tr1::shared_ptr<Step> swing_src,
			 std::tr1::shared_ptr<Step> _swing_dest,
			 Eigen::Matrix3f &fc_Transform);

		
		 void startLeft();
		 void startRight();

		  std::vector<float> getOdoUpdate();
private:
	 inline Kinematics::ChainID getOtherLegChainID();
	   const boost::tuple<const float,const float>getHipHack(const float HYPAngle);
	    void applyHipHacks(float angles[]);
	   void assignStateTimes(std::tr1::shared_ptr<Step> step);
	       const std::vector<float> getStiffnesses();
	bool firstFrame(){return frameCounter == 0;}
	    SupportMode state;
		 unsigned int frameCounter;
		std::tr1::shared_ptr<Step> cur_dest, swing_src, swing_dest,support_step;
		Kinematics::ChainID chainID;
		   const Gait *gait;
		   Eigen::Vector3f goal;		 
		       const float getFootRotation_c();
		   Eigen::Vector3f last_goal;
		   float lastRotation;
		   std::vector<float> odoUpdate;


		   int leg_sign; //-1 для правой
		   std::string leg_name;
		     float lastJoints[Kinematics::LEG_JOINTS];
			 unsigned int doubleSupportFrames;
			     void debugProcessing();
				     void computeOdoUpdate();
					     bool shouldSwitchStates();
						     void switchToNextState();
							 void setState(SupportMode newState);
							     SupportMode nextState();
			 unsigned int singleSupportFrames;
			 unsigned int cycleFrames;
			    const float getEndStepSensorScale();
				    const SensorAngles * sensorAngles;
			 LegJointStiffTuple supporting(Eigen::Matrix3f &fc_Transform);
			    LegJointStiffTuple swinging(Eigen::Matrix3f &fc_Transform);
    const std::vector<float> finalizeJoints(const Eigen::Vector3f & legGoal );
	const boost::tuple <const float, const float > 
		getAnkleAngles();
	 float sensorAngleX, sensorAngleY;
};



#endif //WALKING_LEG_H