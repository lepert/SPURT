#ifndef WalkingArm_h
#define WalkingArm_h
#include "Gait.h"
#include "Kinematics.h"
#include <boost/tuple/tuple.hpp>
#include <vector>
#include "WalkingConstants.h"
#include "Step.h"

typedef boost::tuple<std::vector<float>,
	std::vector<float> > ArmJointStiffTuple;

class WalkingArm
{
public:
	 WalkingArm(const Gait * _gait,Kinematics::ChainID id);
	~WalkingArm(void);
	void startLeft();
	void startRight();
	void setState(SupportMode newState);
	 ArmJointStiffTuple tick(std::tr1::shared_ptr<Step> step);
private:
	SupportMode nextState();
	bool shouldSwitchStates();
	void switchToNextState();
	SupportMode state;
	Kinematics::ChainID chainID;
	const Gait *gait;
	const float getShoulderPitchAddition(std::tr1::shared_ptr<Step> supportStep);

	unsigned int frameCounter;
	unsigned int singleSupportFrames;
	unsigned int doubleSupportFrames;
	bool startStep;
	StepType lastStepType;
};

#endif