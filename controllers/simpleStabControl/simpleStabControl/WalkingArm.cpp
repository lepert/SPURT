#include "WalkingArm.h"




WalkingArm::WalkingArm( const Gait * _gait,Kinematics::ChainID id )
	:state(SUPPORTING),
	chainID(id),
	gait(_gait),
	frameCounter(0),
	startStep(true),
	lastStepType(REGULAR_STEP)
{

}


WalkingArm::~WalkingArm(void)
{
}

void WalkingArm::startLeft()
{
	if(chainID == Kinematics::LARM_CHAIN){

		setState(DOUBLE_SUPPORT);
	}else{
		setState(PERSISTENT_DOUBLE_SUPPORT);
	}
	startStep = true;
	lastStepType = REGULAR_STEP;
}

void WalkingArm::setState( SupportMode newState )
{

	state = newState;
	frameCounter = 0;
}

void WalkingArm::startRight()
{
	if(chainID == Kinematics::LARM_CHAIN){

		setState(PERSISTENT_DOUBLE_SUPPORT);
	}else{
		setState(DOUBLE_SUPPORT);
	}
	startStep = true;
	lastStepType = REGULAR_STEP;
}

ArmJointStiffTuple WalkingArm::tick( std::tr1::shared_ptr<Step> supportStep )
{


	singleSupportFrames = supportStep->singleSupportFrames;
	doubleSupportFrames = supportStep->doubleSupportFrames;
	std::vector<float> armJoints = (chainID == Kinematics::LARM_CHAIN ?
	std::vector<float>(LARM_WALK_ANGLES,&LARM_WALK_ANGLES[Kinematics::ARM_JOINTS]):
	std::vector<float>(RARM_WALK_ANGLES,&RARM_WALK_ANGLES[Kinematics::ARM_JOINTS]));
	 armJoints[0] += getShoulderPitchAddition(supportStep);

	 std::vector<float> armStiffnesses(Kinematics::ARM_JOINTS,gait->stiffness[GaitDefaults::ARM]);
	 armStiffnesses[0] = gait->stiffness[GaitDefaults::ARM_PITCH];
	 frameCounter++;
	 for(unsigned int  i = 0; shouldSwitchStates() && i < 2; i++){
		 switchToNextState();
		 lastStepType = supportStep->type;
	 };

	  return ArmJointStiffTuple(armJoints,armStiffnesses);
}

const float WalkingArm::getShoulderPitchAddition( std::tr1::shared_ptr<Step> supportStep )
{

	float direction = 1.0f; //forward = negative
	float percentComplete = 0.0f;
	switch(state){
	case SUPPORTING:

		direction = -1.0f;
		percentComplete = static_cast<float>(frameCounter)/
			static_cast<float>(singleSupportFrames);
		break;
	case SWINGING:

		direction = 1.0f;
		percentComplete = static_cast<float>(frameCounter)/
			static_cast<float>(singleSupportFrames);
		break;
	case DOUBLE_SUPPORT:

		direction = -1.0f;
		percentComplete = 1.0f;
		break;
	case PERSISTENT_DOUBLE_SUPPORT:

		direction = 1.0f;
		percentComplete = 1.0f;
		break;
	}
	float start = -direction*gait->arm[GaitDefaults::AMPLITUDE];
	float end = direction*gait->arm[GaitDefaults::AMPLITUDE];


	if(supportStep->type== END_STEP){
		if(lastStepType == END_STEP){
			start = end = 0.0f;
		}else if(startStep){
			start = 0.0f;
			end = -end;
		}else{
			end = 0.0f;
			start = -start;
		}
	
		percentComplete = static_cast<float>(frameCounter)/
			static_cast<float>(doubleSupportFrames);
	}



	const float theta = percentComplete*2.0f* MMath::M_PI_FLOAT;
	const float percentToDest = MMath::cycloidx(theta)/(2.0f* MMath::M_PI_FLOAT);

	return start + percentToDest*(end - start);
}

bool WalkingArm::shouldSwitchStates()
{
	switch(state){
	case SUPPORTING:
		return frameCounter >= singleSupportFrames;
	case SWINGING:
		return frameCounter >= singleSupportFrames;
	case DOUBLE_SUPPORT:
		return frameCounter >= doubleSupportFrames;
	case PERSISTENT_DOUBLE_SUPPORT:
		return frameCounter >= doubleSupportFrames;
	}

	throw "Non existent state";
	return false;
}

void WalkingArm::switchToNextState(){
	setState(nextState());
	startStep = false;
}

SupportMode WalkingArm::nextState()
{
	switch(state){
	case SUPPORTING:
		return DOUBLE_SUPPORT;
	case SWINGING:
		return PERSISTENT_DOUBLE_SUPPORT;
	case DOUBLE_SUPPORT:
		return SWINGING;
	case PERSISTENT_DOUBLE_SUPPORT:
		return SUPPORTING;
	default:
		throw "Non existent state";
		return PERSISTENT_DOUBLE_SUPPORT;
	}
}
