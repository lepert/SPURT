#include "Step.h"

/*#define   DEBUG_STEPGENERATOR*/


Step::Step( const WalkVector &target,
	const Gait & gait, const Foot _foot,
	const WalkVector &last /*= ZERO_WALKVECTOR*/, 
	const StepType _type /*= REGULAR_STEP*/ )
	: walkVector(target), sOffsetY(gait.stance[GaitDefaults::LEG_SEPARATION_Y]*0.5f),
	foot(_foot),type(_type),zmpd(false)
{
	copyGaitAttributes(gait.step,gait.zmp,gait.stance);
	switch(_type){
	case REGULAR_STEP:
		updateFrameLengths(stepConfig[GaitDefaults::DURATION],
			stepConfig[GaitDefaults::DBL_SUPP_P]);
		break;
	case END_STEP:

		updateFrameLengths(static_cast<float>(Observer::NUM_PREVIEW_FRAMES) *
			MOTION_FRAME_LENGTH_S,
			1.0f);
		break;
	}

#ifdef DEBUG_STEPGENERATOR
	
	std::cout<<"step constructor provide a step"<<std::endl;
	std::cout<<"frame calculation"<<stepDurationFrames
		<<doubleSupportFrames<<singleSupportFrames<<std::endl;
#endif
	


	    setStepSize(target,last);
		 setStepLiftMagnitude();
#ifdef DEBUG_STEPGENERATOR
		
	std::cout<< stepConfig[GaitDefaults::FOOT_LIFT_ANGLE]<<std::endl;
#endif
	




}

Step::Step( const Step & other)
{

	 copyAttributesFromOther(other);

}

Step::Step( const float new_x, const float new_y, const float new_theta, const Step& other )
{


	copyAttributesFromOther(other);
	x = new_x;
	y = new_y;
	theta = new_theta;
}


Step::~Step(void)
{
}

void Step::copyGaitAttributes(const float _step_config[GaitDefaults::LEN_STEP_CONFIG],
	const float _zmp_config[GaitDefaults::LEN_ZMP_CONFIG],
	const float _stance_config[GaitDefaults::LEN_STANCE_CONFIG]){
		memcpy(stepConfig,_step_config,sizeof(float)*GaitDefaults::LEN_STEP_CONFIG);
		memcpy(zmpConfig,_zmp_config,sizeof(float)*GaitDefaults::LEN_ZMP_CONFIG);
		memcpy(stanceConfig,_stance_config,sizeof(float)*GaitDefaults::LEN_STANCE_CONFIG);
}

void Step::updateFrameLengths( const float duration, const float dblSuppF )
{

	////расчет времени на опорную и неопорную стадию
	stepDurationFrames =
		static_cast<unsigned int>( duration / MOTION_FRAME_LENGTH_S);

	doubleSupportFrames =
		static_cast<unsigned int>(duration *dblSuppF / MOTION_FRAME_LENGTH_S);
	singleSupportFrames = stepDurationFrames - doubleSupportFrames;

}

void Step::setStepSize(const WalkVector &target,
		       const WalkVector &last){

  WalkVector new_walk =ZERO_WALKVECTOR;

#ifdef DEBUG_STEPGENERATOR

 printf("Input to setStepSpeed is (%g,%g,%g), (%g,%g,%g), \n",target.x,target.y,target.theta,
	last.x,last.y,last.theta);
#endif
  

  new_walk = elipseClipVelocities(target);

#ifdef DEBUG_STEPGENERATOR
 printf("After vel clipping (%g,%g,%g)\n",new_walk.x,new_walk.y,new_walk.theta); 
  
#endif

  /*установка параметров х у для каждого шага*/

  new_walk = accelClipVelocities(new_walk,last);

#ifdef DEBUG_STEPGENERATOR
 printf("After accel clipping (%g,%g,%g)\n",new_walk.x,new_walk.y,new_walk.theta);
 
#endif

  walkVector = new_walk;

   new_walk = lateralClipVelocities(new_walk);
   
#ifdef DEBUG_STEPGENERATOR

printf("After leg clipping (%g,%g,%g)\n",new_walk.x,new_walk.y,new_walk.theta);

#endif

   /*когда ограничения сделаны
   еобходимо скорости перевести в дистанцию
   для использования в шагах 
   
   */


 
  const float step_x = new_walk.x*stepConfig[GaitDefaults::DURATION];
  const float step_y = new_walk.y*stepConfig[GaitDefaults::DURATION];//*2.0f;
  const float step_theta = new_walk.theta/**stepConfig[GaitDefaults::DURATION]*2.0f*/;
  

  const float leg_sign = (foot==LEFT_FOOT ?
	  1.0f : -1.0f);
  const float computed_x = step_x - sin(std::abs(step_theta)) *sOffsetY;
  const float computed_y = step_y +
	  leg_sign*sOffsetY*cos(step_theta);
  const float computed_theta = step_theta;

  x = computed_x;
  y = computed_y;
  theta = computed_theta;

 


  #ifdef DEBUG_STEPGENERATOR

    std::cout << "Clipped new step to ("<<x<<","<<y<<","<<theta<<")"<<std::endl;

	#endif
}

const WalkVector Step::elipseClipVelocities( const WalkVector & source )
{

#ifdef DEBUG_STEPGENERATOR
	
	std::cout << "Ellipsoid clip input ("<<source.x<<","<<source.y
		<<","<<source.theta<<")"<<std::endl;
#endif


	//Convert velocities to distances, clip them with an ellipse,
	//then convert back to velocities

	const float theta = MMath::safe_atan2(source.y,source.x);
	const float xy_mag = std::sqrt(std::pow(source.y,2) + std::pow(source.x,2));

	const float max_xy_mag = std::sqrt(std::pow(
		stepConfig[GaitDefaults::MAX_VEL_Y]
	*std::sin(theta),2)
		+ std::pow(
		stepConfig[GaitDefaults::MAX_VEL_X]
	*std::cos(theta),2));
	const float rad_to_mm = max_xy_mag / stepConfig[GaitDefaults::MAX_VEL_THETA];

#ifdef DEBUG_STEPGENERATOR	
	std::cout << "xy_mag = " << xy_mag << " converted theta = " << source.theta*rad_to_mm<<std::endl;
#endif
	
	const float phi = MMath::safe_atan2(xy_mag,source.theta*rad_to_mm);
#ifdef DEBUG_STEPGENERATOR
	
	std::cout << "Ellipsoid vel. clipping: theta = "<<theta<<", phi="<<phi<<std::endl;

#endif
	
	float forward_max =0.0f;
	if(source.x > 0)
		forward_max = std::abs(stepConfig[GaitDefaults::MAX_VEL_X]
	*std::cos(theta)
		*std::sin(phi));
	else
		forward_max = std::abs(stepConfig[GaitDefaults::MIN_VEL_X]
	*std::cos(theta)
		*std::sin(phi));

	const float horizontal_max =
		std::abs(stepConfig[GaitDefaults::MAX_VEL_Y]
	*std::sin(theta)
		*std::sin(phi));

	const float turning_max =
		std::abs(stepConfig[GaitDefaults::MAX_VEL_THETA]
	*std::cos(phi));
#ifdef DEBUG_STEPGENERATOR
	
 std::cout << "Clipping y="<<source.y<<" according to"<<horizontal_max<<std::endl;
#endif
	
	const float new_y_vel = MMath::clip(source.y,horizontal_max);
#ifdef DEBUG_STEPGENERATOR
	
std::cout << "Clipping x="<<source.x<<" according to"<<forward_max<<std::endl;

#endif
		const float new_x_vel = MMath::clip(source.x,forward_max);
#ifdef DEBUG_STEPGENERATOR
	
		std::cout << "Clipping theta="<<source.theta<<" according to"<<turning_max<<std::endl;

#endif	
		const float new_theta_vel = MMath::clip(source.theta,turning_max);


	const WalkVector clippedVelocity = {new_x_vel,new_y_vel,new_theta_vel};
#ifdef DEBUG_STEPGENERATOR

	std::cout << "Ellipsoid clip output ("<<clippedVelocity.x<<","<<clippedVelocity.y
		<<","<<clippedVelocity.theta<<")"<<std::endl;
#endif		

	return clippedVelocity;
}

void Step::setStepLiftMagnitude()
{
	const float percent_of_forward_max  = MMath::clip(x,0,stepConfig[GaitDefaults::MAX_VEL_X])
		/ stepConfig[GaitDefaults::MAX_VEL_X];

	stepConfig[GaitDefaults::FOOT_LIFT_ANGLE] *= percent_of_forward_max;
}

void Step::copyAttributesFromOther( const Step &other )
{

	x = other.x;
	y = other.y;
	theta = other.theta;
	walkVector = other.walkVector;
	stepDurationFrames = other.stepDurationFrames;
	doubleSupportFrames = other.doubleSupportFrames;
	singleSupportFrames = other.singleSupportFrames;
	sOffsetY= other.sOffsetY;
	foot = other.foot;
	type = other.type;
	zmpd = other.zmpd;
	copyGaitAttributes(other.stepConfig,other.zmpConfig,other.stanceConfig);

}

const WalkVector Step::accelClipVelocities( const WalkVector & source, const WalkVector & last )
{
 
	WalkVector result = source;
	/*срежем скорости если это еще  не остановка*/
	if(source.x != 0.0f ||  source.y != 0.0f || source.theta!= 0.0f){
		result.x = MMath::clip(source.x,
			last.x - stepConfig[GaitDefaults::MAX_ACC_X],
			last.x + stepConfig[GaitDefaults::MAX_ACC_X]);
		result.y = MMath::clip(source.y,
			last.y - stepConfig[GaitDefaults::MAX_ACC_Y]*0.5f,
			last.y + stepConfig[GaitDefaults::MAX_ACC_Y]*0.5f);
		result.theta = MMath::clip(source.theta,
			last.theta - stepConfig[GaitDefaults::MAX_ACC_THETA]*0.5f,
			last.theta + stepConfig[GaitDefaults::MAX_ACC_THETA]*0.5f);
	}
	return result;

}

const WalkVector Step::lateralClipVelocities( const WalkVector & source )
{

	WalkVector result = source;


	if(result.y > 0){
		if(!foot == LEFT_FOOT){
			result.y = 0.0f;
		}
	}else if(result.y < 0){
		if(foot == LEFT_FOOT){
			result.y = 0.0f;
		}
	}

	if(result.theta > 0){
		if(!foot == LEFT_FOOT){
			result.theta = 0.0f;
		}
	}else if (result.theta < 0){
		if(foot == LEFT_FOOT){
			result.theta = 0.0f;
		}
	}

	return result;

}


