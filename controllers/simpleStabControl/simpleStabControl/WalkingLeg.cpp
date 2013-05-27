#include "WalkingLeg.h"


WalkingLeg::WalkingLeg(
	const Gait *_gait,const SensorAngles * _sensorAngles,
	Kinematics::ChainID id)
	:state(SUPPORTING),
	frameCounter(0),
	cur_dest(EMPTY_STEP),swing_src(EMPTY_STEP),swing_dest(EMPTY_STEP),
	support_step(EMPTY_STEP),
	chainID(id), gait(_gait),
	goal(Eigen::Vector3f(0.0f,0.0f,0.0f)),
	last_goal(Eigen::Vector3f(0.0f,0.0f,0.0f)),
	lastRotation(0.0f),odoUpdate(3,0.0f),
	sensorAngles(_sensorAngles),
	leg_sign(id == Kinematics::LLEG_CHAIN ? 1 : -1),
	leg_name(id == Kinematics::LLEG_CHAIN ? "left" : "right")
{

	for ( unsigned int i = 0 ; i< Kinematics::LEG_JOINTS; i++) lastJoints[i]=0.0f;
}




void WalkingLeg::startLeft(){
	if(chainID == Kinematics::LLEG_CHAIN){
		//we will start walking first by swinging left leg (this leg), so
		//we want double, not persistent, support
		setState(DOUBLE_SUPPORT);
	}else{
		setState(PERSISTENT_DOUBLE_SUPPORT);
	}
}
void WalkingLeg::startRight(){
	if(chainID == Kinematics::LLEG_CHAIN){
		//we will start walking first by swinging right leg (not this leg), so
		//we want persistent double support
		setState(PERSISTENT_DOUBLE_SUPPORT);
	}else{
		setState(DOUBLE_SUPPORT);
	}
}



WalkingLeg::~WalkingLeg(){

}

void WalkingLeg::setSteps( std::tr1::shared_ptr<Step> _swing_src, std::tr1::shared_ptr<Step> _swing_dest, std::tr1::shared_ptr<Step> _suppoting )
{

	swing_src = _swing_src;
	swing_dest = _swing_dest;
	support_step = _suppoting;
	assignStateTimes(support_step);
}

void WalkingLeg::assignStateTimes( std::tr1::shared_ptr<Step> step )
{
	doubleSupportFrames = step->doubleSupportFrames;
	singleSupportFrames = step->singleSupportFrames;
	cycleFrames = step->stepDurationFrames;
}

const float WalkingLeg::getFootRotation()
{
	if(state != SUPPORTING && state != SWINGING)
		return swing_src->theta;

	const float percent_complete =
		static_cast<float>(frameCounter) /
		static_cast<float>(singleSupportFrames);

	const float theta = percent_complete*2.0f*M_PI_FLOAT;
	const float percent_to_dest = MMath::cycloidx(theta)/(2.0f*M_PI_FLOAT);

	const float end = swing_dest->theta;
	const float start = swing_src->theta;

	const float value = start + (end-start)*percent_to_dest;
	return value;
}

LegJointStiffTuple WalkingLeg::tick( std::tr1::shared_ptr<Step> step, std::tr1::shared_ptr<Step> _swing_src, std::tr1::shared_ptr<Step> _swing_dest, Eigen::Matrix3f &fc_Transform )
{

#ifdef DEBUG_WALKINGLEG
	cout << "WalkingLeg::tick() "<<leg_name <<" leg, state is "<<state<<endl;
#endif


	cur_dest = step;
	swing_src = _swing_src;
	swing_dest = _swing_dest;

	LegJointStiffTuple result;

	switch(state){
	case SUPPORTING:
		result  = supporting(fc_Transform);
		break;
	case SWINGING:
		result  =  swinging(fc_Transform);
		break;
	case DOUBLE_SUPPORT:

		cur_dest = swing_src;
		result  = supporting(fc_Transform);
		break;
	case PERSISTENT_DOUBLE_SUPPORT:
		result  = supporting(fc_Transform);
		break;
	default:
		std::cout << "Invalid SupportMode"<<std::endl;
		throw "Invalid SupportMode passed to WalkingLeg::tick";
	}



	debugProcessing();
	computeOdoUpdate();
	last_goal = goal;
	lastRotation = getFootRotation();
	frameCounter++;

	for(unsigned int  i = 0; shouldSwitchStates() && i < 2; i++,switchToNextState());

	return result;


}

LegJointStiffTuple WalkingLeg::supporting( Eigen::Matrix3f &fc_Transform )
{
	/*std::cout << "Support step is " <<std::endl;*/


    Eigen::Vector3f dest_f (cur_dest->x,cur_dest->y,1);

	   
		Eigen::Vector3f dest_c = (fc_Transform*dest_f);
		float dest_x = dest_c(0);
		float dest_y = dest_c(1);

		goal(0) = dest_x; //targetX 
		goal(1) = dest_y;  //targetY
		goal(2) = -gait->stance[GaitDefaults::BODY_HEIGHT];         //targetZ

		int i=0;

		   std::vector<float> joint_result = finalizeJoints(goal);
		   std::vector<float> stiff_result = getStiffnesses();

		 return LegJointStiffTuple(joint_result,stiff_result);

}

const std::vector<float> WalkingLeg::finalizeJoints( const Eigen::Vector3f & footGoal )
{
	 const float startStopSensorScale = getEndStepSensorScale();
	
#ifdef USE_COM_CONTROL
	 const float COM_SCALE = startStopSensorScale;
	 const ufvector4 com_c = Kinematics::getCOMc(sensors->getMotionBodyAngles());
#else
	 const float COM_SCALE = startStopSensorScale;
	 const Eigen::Vector4f com_c(0,0,0,1);
#endif

	
	 const float COM_Z_OFF = 69.9f;
	 Eigen::Vector3f comFootGoal = footGoal;
	 comFootGoal(2) += COM_Z_OFF*COM_SCALE;


	 const boost::tuple <const float, const float > sensorCompensation =
		 sensorAngles->getAngles(startStopSensorScale);


	 /* для того чтобы тело робота двигалось в направлении хотьбы используется 
	 фильтр калмана*/
	 const float bodyAngleX = sensorAngleX =
		 sensorCompensation.get<SensorAngles::X>();
	 const float bodyAngleY = sensorAngleY = gait->stance[GaitDefaults::BODY_ROT_Y] +
		 sensorCompensation.get<SensorAngles::Y>();



/*
	 std::cout<<"sensorAngleX*MMath::TO_DEG"<<sensorAngleX*MMath::TO_DEG<<std::endl;
	 std::cout<<"sensorAngleY"<<sensorAngleY*MMath::TO_DEG<<std::endl;*/

	 const boost::tuple <const float, const float > ankleAngleCompensation =
		 getAnkleAngles();
	 const float footAngleX = ankleAngleCompensation.get<0>();
	 const float footAngleY = ankleAngleCompensation.get<1>();
	 const float footAngleZ = getFootRotation_c()
		 + leg_sign*gait->stance[GaitDefaults::LEG_ROT_Z]*0.5;

	 const Eigen::Vector3f bodyOrientation(bodyAngleX, bodyAngleY, 0.0f);
	 const Eigen::Vector3f footOrientation (footAngleX, footAngleY, footAngleZ);

	 const Eigen::Vector3f bodyGoal ( -com_c(0)*COM_SCALE,
		 -com_c(1)*COM_SCALE,
		 COM_Z_OFF*COM_SCALE);

	 Kinematics::IKLegResult result =
		 Kinematics::legIK(chainID,comFootGoal,footOrientation,
		 bodyGoal,bodyOrientation);

 //applyHipHacks(result.angles);

 memcpy(lastJoints, result.angles, LEG_JOINTS*sizeof(float));
 return vector<float>(result.angles, &result.angles[LEG_JOINTS]);



}

const float WalkingLeg::getEndStepSensorScale()
{
	
	if(support_step->type == REGULAR_STEP)
		return 1.0f;

	if ( swing_src->type==END_STEP)
		
		return 0.0f;

	float startScale, endScale;
	if(swing_dest->type == REGULAR_STEP){
		
		startScale = 0.0f;
		endScale = 1.0f;
	}else{
		
		startScale = 1.0f;
		endScale = 0.0f;
	}
	
	float percent_complete = static_cast<float>(frameCounter) /
		static_cast<float>(doubleSupportFrames);

	const float theta = percent_complete*2.0f*M_PI_FLOAT;//TODO: move to common
	const float percent_to_dest = MMath::cycloidx(theta)/(2.0f*M_PI_FLOAT);

	return startScale + (endScale-startScale)*percent_to_dest;

}

const boost::tuple <const float, const float > WalkingLeg::getAnkleAngles()
{


	if(state != SWINGING){
		return boost::tuple<const float, const float>(0.0f,0.0f);
	}

	const float angle = static_cast<float>(frameCounter)/
		static_cast<float>(singleSupportFrames)*M_PI_FLOAT;

	const float scale = std::sin(angle);


	const float ANKLE_LIFT_ANGLE = swing_dest->stepConfig[GaitDefaults::FOOT_LIFT_ANGLE]*scale;

	return boost::tuple<const float, const float>(0.0f,ANKLE_LIFT_ANGLE);
}



const float WalkingLeg::getFootRotation_c()
{
	const float abs_rot =  std::abs(getFootRotation());

	const float rot_rel_c = abs_rot*0.5*leg_sign;

	return rot_rel_c;
}

void WalkingLeg::applyHipHacks( float angles[] )
{
	const float footAngleZ = getFootRotation_c();
	boost::tuple <const float, const float > hipHacks  = getHipHack(footAngleZ);
	angles[1] += hipHacks.get<1>(); //HipRoll
	angles[2] += hipHacks.get<0>(); //HipPitch
}



const boost::tuple<const float,const float> WalkingLeg::getHipHack( const float footAngleZ )
{

	ChainID hack_chain;
	if(state == SUPPORTING){
		hack_chain = chainID;
	}else if(state == SWINGING){
		hack_chain = getOtherLegChainID();
	}else{
		// This step is double support, returning 0 hip hack
		return 0.0f;
	}
	const float support_sign = (state !=SWINGING? 1.0f : -1.0f);
	const float absFootAngle = std::abs(footAngleZ);

	//Calculate the compensation to the HIPROLL
	float MAX_HIP_ANGLE_OFFSET = (hack_chain == LLEG_CHAIN ?
		gait->hack[GaitDefaults::L_HIP_AMP]:
	gait->hack[GaitDefaults::R_HIP_AMP]);


	static int stage;
	if (firstFrame()) stage = 0;

	float hr_offset = 0.0f;

	if (stage == 0) { // we are rising
		// we want to raise the foot up for the first third of the step duration
		hr_offset = MAX_HIP_ANGLE_OFFSET*
			static_cast<float>(frameCounter) /
			(static_cast<float>(singleSupportFrames)/3.0f);
		if (frameCounter >= (static_cast<float>(singleSupportFrames)
			/ 3.0f) )
			stage++;

	}
	else if (stage == 1) { // keep it level
		hr_offset  = MAX_HIP_ANGLE_OFFSET;

		if (frameCounter >= 2.* static_cast<float>(singleSupportFrames)/3)
			stage++;
	}
	else {// stage 2, set the foot back down on the ground
		hr_offset = max(0.0f,
			MAX_HIP_ANGLE_OFFSET*
			static_cast<float>(singleSupportFrames
			-frameCounter)/
			( static_cast<float>(singleSupportFrames)/
			3.0f) );
	}

	const float hipPitchAdjustment = -hr_offset * std::sin(footAngleZ);
	const float hipRollAdjustment = support_sign*(hr_offset *
		static_cast<float>(leg_sign)*
		std::cos(footAngleZ) );

	return boost::tuple<const float, const float> (hipPitchAdjustment,
		hipRollAdjustment);


}
inline ChainID WalkingLeg::getOtherLegChainID(){
	return (chainID==LLEG_CHAIN ?
RLEG_CHAIN : LLEG_CHAIN);
}

const std::vector<float> WalkingLeg::getStiffnesses()
{

	const float maxS = gait->stiffness[GaitDefaults::HIP];
	const float anklePitchS = gait->stiffness[GaitDefaults::AP];
	const float ankleRollS = gait->stiffness[GaitDefaults::AR];
	const float kneeS = gait->stiffness[GaitDefaults::KP];

	float stiffnesses[LEG_JOINTS] = {maxS, maxS, maxS,
		kneeS,anklePitchS,ankleRollS};
	vector<float> stiff_result = vector<float>(stiffnesses,
		&stiffnesses[LEG_JOINTS]);
	return stiff_result;
}

LegJointStiffTuple WalkingLeg::swinging( Eigen::Matrix3f &fc_Transform )
{
	/*std::cout << "swinging step is " <<std::endl;*/

	Eigen::Vector3f dest_f (cur_dest->x,cur_dest->y,1);
	Eigen::Vector3f  src_f (swing_src->x,swing_src->y,1);

	Eigen::Vector3f dest_c = (fc_Transform*dest_f);
	Eigen::Vector3f src_c = (fc_Transform*src_f);

	static float dist_to_cover_x = 0;
	static float dist_to_cover_y = 0;

	if(firstFrame()){
		dist_to_cover_x = cur_dest->x - swing_src->x;
		dist_to_cover_y = cur_dest->y - swing_src->y;
	}


	float percent_complete =
		( static_cast<float>(frameCounter) /
		static_cast<float>(singleSupportFrames));

	float theta = percent_complete*2.0f*M_PI_FLOAT;
	float stepHeight = gait->step[GaitDefaults::STEP_HEIGHT];
	float percent_to_dest_horizontal = MMath::cycloidx(theta)/(2.0f*M_PI_FLOAT);

	
	float dest_x = src_f(0) + percent_to_dest_horizontal*dist_to_cover_x;
	float dest_y = src_f(1) + percent_to_dest_horizontal*dist_to_cover_y;

	Eigen::Vector3f target_f(dest_x,dest_y,1);
		Eigen::Vector3f target_c = (fc_Transform* target_f);

		float target_c_x = target_c(0);
		float target_c_y = target_c(1);

		float radius =gait->step[GaitDefaults::STEP_HEIGHT]/2;
		float heightOffGround = radius*MMath::cycloidy(theta);

		goal(0) = target_c_x;
		goal(1) = target_c_y;
		goal(2) = -gait->stance[GaitDefaults::BODY_HEIGHT] + heightOffGround;

		vector<float> joint_result = finalizeJoints(goal);

		vector<float> stiff_result = getStiffnesses();
		return LegJointStiffTuple(joint_result,stiff_result);

}

void WalkingLeg::debugProcessing()
{
#ifdef DEBUG_WALKING_STATE_TRANSITIONS
	if (firstFrame()){
		if(chainID == LLEG_CHAIN){
			cout<<"Left leg "
		}else{
			cout<<"Right leg "
		}
		if(state == SUPPORTING)
			cout <<"switched into single support"<<endl;
		else if(state== DOUBLE_SUPPORT || state == PERSISTENT_DOUBLE_SUPPORT)
			cout <<"switched into double support"<<endl;
		else if(state == SWINGING)
			cout << "switched into swinging."<<endl;
	}
#endif


}

void WalkingLeg::computeOdoUpdate()
{
	const float thetaDiff = getFootRotation() - lastRotation;
	
	const float thetaCOMMovement = -thetaDiff*0.33f; //.33 is somewhat experimental

	const Eigen::Vector3f diff = goal-last_goal;
	const float xCOMMovement = -diff(0);
	const float yCOMMovement = -diff(1);

	odoUpdate[0] =xCOMMovement*gait->odo[GaitDefaults::X_SCALE];
	odoUpdate[1] = yCOMMovement*gait->odo[GaitDefaults::Y_SCALE];
	odoUpdate[2] = thetaCOMMovement*gait->odo[GaitDefaults::THETA_SCALE];
}

bool WalkingLeg::shouldSwitchStates()
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

void WalkingLeg::switchToNextState()
{
	 setState(nextState());
}

void WalkingLeg::setState( SupportMode newState )
{  
	state = newState;
frameCounter = 0;
if(state == PERSISTENT_DOUBLE_SUPPORT ||
	state == DOUBLE_SUPPORT)
	lastRotation = -lastRotation;

}


SupportMode WalkingLeg::nextState()
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

std::vector<float> WalkingLeg::getOdoUpdate()
{
	return odoUpdate;
}
