#include "StepGenerator.h"
#include <iostream>

StepGenerator::StepGenerator(std::tr1::shared_ptr<GeneralSensors> s)
	:mController_x(new Observer()),
	mController_y(new Observer()),
	sensorAngles(&DEFAULT_GAIT,s),
	gait(DEFAULT_GAIT),
	done(true),
	currentZMPDSteps(),
	zmp_ref_x(std::list<float>()),zmp_ref_y(std::list<float>()),
	si_Transform(Eigen::Matrix3f::Identity()),
	last_zmp_end_s(Eigen::Vector3f(0.0f,0.0f,1.0f)),
	est_zmp_i(Eigen::Vector3f(0.0f,0.0f,1.0f)),
	zmp_filter(),
	controller_x(new Observer()),
	controller_y(new Observer()),
	leftLeg(&gait,&sensorAngles,LLEG_CHAIN),
	leftArm(&gait,LARM_CHAIN), rightArm(&gait,RARM_CHAIN),
	rightLeg(&gait,&sensorAngles,RLEG_CHAIN),
	 supportFoot(LEFT_SUPPORT),
	 counter(0),
	 com_i(Eigen::Vector3f(0.0f,0.0f,1.0f))
{
	
}


StepGenerator::~StepGenerator(void)
{
	delete controller_x; delete controller_y;
}

void StepGenerator::takeSteps( const float _x, const float _y, const float _theta, const int _numSteps )
{

#ifdef DEBUG_STEPGENERATOR
	std::cout << "takeSteps called with (" << _x << "," << _y<<","<<_theta
		<<") and with nsteps = "<<_numSteps<<std::endl;
#endif


	
	/**done=true  в tickleg*/
	if(!done){
		std::cout<< "Warning!!! Step Command with (" << _x << "," << _y<<","<<_theta
			<<") and with "<<_numSteps<<" Steps were APPENDED because"
			"StepGenerator is already active!!" <<std::endl;
	}else{


		resetQueues();

		
	
		const bool startLeft = decideStartLeft(_y,_theta);
				
		

		/*указываем что начало ходьбы с левой ноги*/

	
	
			resetSteps(startLeft);
		
	
		
		//Adding this step is necessary because it was not added in the start left right
#ifdef DEBUG_STEPGENERATOR
		std::cout<<"takeStepsGenerateFirstStep"<<std::endl;
#endif
		
		generateStep(_x, _y, _theta);
		

		done = false;

	}

	

	for(int i =0; i < _numSteps; i++){

#ifdef DEBUG_STEPGENERATOR
		std::cout<<"takeStepsGenerateActualNumberOfSteps"<<std::endl;
#endif
		
		
		generateStep(_x, _y, _theta);
	}


	/*
	когда истекает очередь шагов
	tick_controller генерирует 
	последующие шаги-используя
	единственные оставшиеся значения 
	представленные ниже
	*/
	x = 0.0f; y =0.0f; theta = 0.0f;



}

void StepGenerator::resetQueues()
{
	futureSteps.clear();
	currentZMPDSteps.clear();
	zmp_ref_x.clear();
	zmp_ref_y.clear();

}


const bool StepGenerator::decideStartLeft(const float lateralVelocity,
	const float radialVelocity){
 
    if(lateralVelocity == 0.0f){
        return radialVelocity > 0.0f;
    }
    return lateralVelocity > 0.0f;
   
}

void StepGenerator::resetSteps( const bool startLeft )
{


	

	controller_x->initState(gait.stance[GaitDefaults::BODY_OFF_X],0.0f,
		gait.stance[GaitDefaults::BODY_OFF_X]);
	controller_y->initState(0.0f,0.0f,0.0f);
	zmp_filter = ZmpEKF();
	sensorAngles.reset();

	last_zmp_end_s=Eigen::Vector3f(0.0f,0.0f,1.0f);
	si_Transform =  Eigen::Matrix3f::Identity();
	
		
	
	Foot dummyFoot = LEFT_FOOT;
	Foot firstSupportFoot = RIGHT_FOOT;
	    float supportSign = 1.0f;

		

		if(startLeft){
#ifdef DEBUG_STEPGENERATOR
			cout << "StepGenerator::startLeft"<<endl;
#endif


			leftLeg.startRight();
			rightLeg.startRight();
			leftArm.startRight();
			rightArm.startRight();

			dummyFoot = RIGHT_FOOT;
			firstSupportFoot = LEFT_FOOT;
			supportSign = 1.0f;
			nextStepIsLeft = false;

		}else{ //startRight
#ifdef DEBUG_STEPGENERATOR
			cout << "StepGenerator::startRight"<<endl;
#endif


			leftLeg.startLeft();
			rightLeg.startLeft();
			leftArm.startLeft();
			rightArm.startLeft();

		
			dummyFoot = LEFT_FOOT;
			firstSupportFoot = RIGHT_FOOT;
			supportSign = -1.0f;
			nextStepIsLeft = true;

		}


		
		const Eigen::Matrix3f initStart =
			Kinematics::translation3D(0.0f,
			supportSign*(HIP_OFFSET_Y));

		
		if_Transform=initStart;


	
		resetOdometry(gait.stance[GaitDefaults::BODY_OFF_X],-supportSign*HIP_OFFSET_Y);




#ifdef DEBUG_STEPGENERATOR
	std::cout<<"resetStepsGenerateStep"<<std::endl;
#endif

	std::tr1::shared_ptr<Step> firstSupportStep =
		std::tr1::shared_ptr<Step>(new Step(ZERO_WALKVECTOR,
		gait,
		firstSupportFoot,
		ZERO_WALKVECTOR,
		END_STEP)); 
	std::tr1::shared_ptr<Step> dummyStep =
		shared_ptr<Step>(new Step(ZERO_WALKVECTOR,
		gait,
		dummyFoot));

	currentZMPDSteps.push_back(dummyStep);
	    fillZMP(firstSupportStep);
		currentZMPDSteps.push_back(firstSupportStep);
		lastQueuedStep = firstSupportStep;

	lastQueuedStep = firstSupportStep;
	
}


void StepGenerator::resetOdometry(const float initX, const float initY){
    cc_Transform = Kinematics::translation3D(-initX,-initY);
}

void StepGenerator::generateStep( float _x,float _y, float _theta )
{
	StepType type;
	
	/*проверим на то что это конечный шаг*/

	if(gait.step[GaitDefaults::WALKING] == GaitDefaults::NON_WALKING_GAIT){
		type = END_STEP;
		_x = 0.0f;
		_y = 0.0f;
		_theta = 0.0f;

	}else if (_x ==0 && _y == 0 && _theta == 0){

		type = END_STEP;

	}else{

		/*
		когда опустошается контейнер шагов
		значения х у тета обнуляются
		когда после  END_STEP 
		параметры шага обнуляются

		
		
		*/
		if(lastQueuedStep->type == END_STEP){
			if (lastQueuedStep->zmpd){
				type = REGULAR_STEP;
				_x = 0.0f;
				_y = 0.0f;
				_theta = 0.0f;


			}else{
				type = REGULAR_STEP;
				lastQueuedStep->type = REGULAR_STEP;
			}
		}else{
			
			type = REGULAR_STEP;
		}
	}

	const WalkVector new_walk = {_x,_y,_theta};


	std::tr1::shared_ptr<Step> step(new Step(new_walk,
		                                     gait,
		                                      (nextStepIsLeft ?
                                             LEFT_FOOT : RIGHT_FOOT),
			                                 lastQueuedStep->walkVector,
			                                 type));

#ifdef DEBUG_STEPGENERATOR

std::cout << "Generated a new step: " <<std::endl;
#endif

		futureSteps.push_back(step);
		lastQueuedStep = step;
		
		nextStepIsLeft = !nextStepIsLeft;
		


}

void StepGenerator::setSpeed( const float _x, const float _y, const float _theta )
{
	clearFutureSteps();

	x = _x;
	y = _y;
	theta = _theta;



	if(done){

		resetQueues();

		
		const bool startLeft = decideStartLeft(y,theta);
		resetSteps(startLeft);
	}
	done = false;
}

void StepGenerator::clearFutureSteps(){
	
	futureSteps.clear();
	if(currentZMPDSteps.size() > 0){
		lastQueuedStep = currentZMPDSteps.back();
		nextStepIsLeft = (lastQueuedStep->foot != LEFT_FOOT);
	}
}

void StepGenerator::tick_controller()
{

	counter++;
#ifdef ZMP_DEBUG
	std::cout << "StepGenerator::tick_controller" << std::endl;
#endif

	/*нахождение ссылочной змп на полу
	это разметка точками траектории таза робота 
	для каждой фазы передвижения*/

	zmp_xy_tuple zmp_ref = generate_zmp_ref();

	

	const float cur_zmp_ref_x =  zmp_ref_x.front();
	const float cur_zmp_ref_y = zmp_ref_y.front();

	zmp_ref_x.pop_front();
	zmp_ref_y.pop_front();


	
 	est_zmp_i(0) =  cur_zmp_ref_x;
 	est_zmp_i(1) = cur_zmp_ref_y;
	
	//std::cout<<"x"<<cur_zmp_ref_x <<" y"<<cur_zmp_ref_y<<std::endl;

  	const float com_x = controller_x->tick(zmp_ref.get<0>(),cur_zmp_ref_x,
  		est_zmp_i(0));

 	const float com_y = controller_y->tick(zmp_ref.get<1>(),cur_zmp_ref_y,
		est_zmp_i(1));
	
 
#ifdef ZMP_DEBUG
 	std::cout<<"ZMPOUTRef"<<std::endl;
 	std::cout<<"x"<<com_x <<" y"<<com_y<<std::endl;
#endif
 	com_i = Eigen::Vector3f(com_x,com_y,1);


}

zmp_xy_tuple StepGenerator::generate_zmp_ref()
{
	/*генерация ZMP для запуска контроллера(должно быть достаточно шагов)*/
	
	

	while (zmp_ref_y.size() <= Observer::NUM_PREVIEW_FRAMES ||
		// достаточное количество фреймов должно быть
		currentZMPDSteps.size() < MIN_NUM_ENQUEUED_STEPS) {
			if (futureSteps.size() == 0){
				std::cout<<x<<y<<theta;
				
				/*throw;*/
				generateStep(x, y, theta); 



			}
			else {

				std::tr1::shared_ptr<Step> nextStep = futureSteps.front();
				futureSteps.pop_front();

				fillZMP(nextStep);
				
				currentZMPDSteps.push_back(nextStep);

			}
	}
	return zmp_xy_tuple(&zmp_ref_x, &zmp_ref_y);
}

void StepGenerator::fillZMP( const std::tr1::shared_ptr<Step> newSupportStep )
{

	switch(newSupportStep->type){
	case REGULAR_STEP:

		fillZMPRegular(newSupportStep);
		break;
	case END_STEP: 

		fillZMPEnd(newSupportStep);
		break;
	default:
		throw "Unsupported Step type";
	}

	 newSupportStep->zmpd = true;

}



void StepGenerator::fillZMPRegular( const std::tr1::shared_ptr<Step> newSupportStep )
{
	const float sign = (newSupportStep->foot == LEFT_FOOT ? 1.0f : -1.0f);
	const float last_sign = -sign;

	
	float X_ZMP_FOOT_LENGTH = 0.0f;// HACK/TODO make this center foot gait->footLengthX;

	
	/*хак  в том что перенося вес с 
	открытой ноги робот переводит себя 
	в неустойчивое положение
	поэтому ему надо раскачиваться больше*/

	const float HACK_AMOUNT_PER_PI_OF_TURN =
		newSupportStep->zmpConfig[GaitDefaults::TURN_ZMP_OFF];
	const float HACK_AMOUNT_PER_1_OF_LATERAL =
		newSupportStep->zmpConfig[GaitDefaults::STRAFE_ZMP_OFF];

	float adjustment = ((newSupportStep->theta / MMath::M_PI_FLOAT)
		* HACK_AMOUNT_PER_PI_OF_TURN);
	adjustment += (newSupportStep->y - (sign*Kinematics::HIP_OFFSET_Y))
		* HACK_AMOUNT_PER_1_OF_LATERAL;

	//zmpстроится только в пределе тела робота
	//чтобы продвинуть его слегка дальше используем


	float Y_ZMP_OFFSET = (newSupportStep->foot == LEFT_FOOT ?
		newSupportStep->zmpConfig[GaitDefaults::L_ZMP_OFF_Y]:
	newSupportStep->zmpConfig[GaitDefaults::R_ZMP_OFF_Y]);

	// When we turn, the ZMP offset needs to be corrected for the rotation of
	// newSupportStep. A picture would be very useful here. Someday...
	float y_zmp_offset_x = -sin(std::abs(newSupportStep->theta)) * Y_ZMP_OFFSET;
	float y_zmp_offset_y = cos(newSupportStep->theta) * Y_ZMP_OFFSET; 

	//lets define the key points in the s frame. See diagram in paper
	//to use bezier curves, we would need also directions for each point
	const Eigen::Vector3f start_s = last_zmp_end_s;

	const Eigen::Vector3f end_s =
		             Eigen::Vector3f(newSupportStep->x +
		             gait.stance[GaitDefaults::BODY_OFF_X] +
		             y_zmp_offset_x,
		             newSupportStep->y + sign*y_zmp_offset_y,1);
	const Eigen::Vector3f mid_s =
		               Eigen::Vector3f(newSupportStep->x +
		               gait.stance[GaitDefaults::BODY_OFF_X] +
		               y_zmp_offset_x - X_ZMP_FOOT_LENGTH,
		                newSupportStep->y + sign*y_zmp_offset_y,1);
	

	const Eigen::Vector3f start_i = si_Transform*start_s;
 	const Eigen::Vector3f mid_i = si_Transform*mid_s;
 	const Eigen::Vector3f end_i = si_Transform*end_s;




	/**интерполяция между тремя точками
	линия между start и mid -диагональ 
	в течении которой робот в стадии 
	двойной поддержки  тоже и при переходе midle end*/

	/**двойная поддержка состоит из трех фаз
	1)статич в starti
	2)диагональное движение мд s m
	3)стат в e
	время разбито меж фазами см gait->dblSupInactivePercentage
	*/
	const int halfNumDSChops = //DS - DoubleStaticChops
		static_cast<int>(static_cast<float>(newSupportStep->doubleSupportFrames)*
		newSupportStep->zmpConfig[GaitDefaults::DBL_SUP_STATIC_P]/2.0f);
	const int numDMChops = //DM - DoubleMovingChops
		newSupportStep->doubleSupportFrames - halfNumDSChops*2;
	/*фаза 1 останься на месте*/
	for(int i = 0; i< halfNumDSChops; i++){
		zmp_ref_x.push_back(start_i(0));
		zmp_ref_y.push_back(start_i(1));
	}
	/*фаза 2 */
	
	for(int i = 0; i< numDMChops; i++){
		Eigen::Vector3f new_i = start_i +
			(static_cast<float>(i)/
			static_cast<float>(numDMChops) ) *
			(mid_i-start_i);

		zmp_ref_x.push_back(new_i(0));
		zmp_ref_y.push_back(new_i(1));
	}

	// mid_i
	for(int i = 0; i< halfNumDSChops; i++){
		zmp_ref_x.push_back(mid_i(0));
		zmp_ref_y.push_back(mid_i(1));
	}

	/*поддержка одной ноги останемся в зоне этой ноги*/

	const int numSChops = newSupportStep->singleSupportFrames;
	for(int i = 0; i< numSChops; i++){
		//    const int numSChops = gait->stepDurationFrames;
		//    for(int i = 0; i< gait->stepDurationFrames; i++){

		Eigen::Vector3f new_i = mid_i +
			(static_cast<float>(i) /
			static_cast<float>(numSChops) ) *
			(end_i-mid_i);

		zmp_ref_x.push_back(new_i(0));
		zmp_ref_y.push_back(new_i(1));
	}

	
	si_Transform = si_Transform*get_s_sprime(newSupportStep);

	last_zmp_end_s = (get_sprime_s(newSupportStep))*end_s;


}

const Eigen::Matrix3f StepGenerator::get_sprime_s( const std::tr1::shared_ptr<Step> step )
{

	const float leg_sign = (step->foot == LEFT_FOOT ? 1.0f : -1.0f);

	const float x = step->x;
	const float y = step->y;
	const float theta = step->theta;


	const Eigen::Matrix3f trans_f_s =
		Kinematics::translation3D(0,leg_sign*Kinematics::HIP_OFFSET_Y);

	const Eigen::Matrix3f trans_sprime_f =
		(Kinematics::rotation3D(Kinematics::Z_AXIS,-theta))*
		(Kinematics::translation3D(-x,-y));
// 
// 	std::cout<<"Kinematics::rotation3D(Kinematics::Z_AXIS,-theta)"
// 		<<Kinematics::rotation3D(Kinematics::Z_AXIS,-theta)<<std::endl;
// 	std::cout<<"Kinematics::translation3D(-x,-y)"
// 		<<Kinematics::translation3D(-x,-y)<<std::endl;
	return trans_f_s*trans_sprime_f;

}

const Eigen::Matrix3f StepGenerator::get_s_sprime( const std::tr1::shared_ptr<Step> step )
{
	const float leg_sign = (step->foot == LEFT_FOOT ? 1.0f : -1.0f);

	const float x = step->x;
	const float y = step->y;
	const float theta = step->theta;

	const Eigen::Matrix3f trans_f_s =
		Kinematics::translation3D(0,-leg_sign*Kinematics::HIP_OFFSET_Y);

	const Eigen::Matrix3f trans_sprime_f =
		(Kinematics::translation3D(x,y))*(
		Kinematics::rotation3D(Kinematics::Z_AXIS,theta));
	return trans_sprime_f*trans_f_s;
}

void StepGenerator::fillZMPEnd( const std::tr1::shared_ptr<Step> newSupportStep )
{

	const Eigen::Vector3f end_s =
		Eigen::Vector3f(gait.stance[GaitDefaults::BODY_OFF_X],
		0.0f,1.0f);
	const Eigen::Vector3f end_i = si_Transform*end_s;
	
	for (unsigned int i = 0; i < newSupportStep->stepDurationFrames; i++){
		zmp_ref_x.push_back(end_i(0));
		zmp_ref_y.push_back(end_i(1));
	}

	
	last_zmp_end_s = (get_sprime_s(newSupportStep))*end_s;

}

float StepGenerator::scaleSensors( const float sensorZMP, const float perfectZMP )
{
	const float sensorWeight = 0.0f;//gait->sensor[WP::OBSERVER_SCALE];
	return sensorZMP*sensorWeight + (1.0f - sensorWeight)*perfectZMP;
}

WalkLegsTuple StepGenerator::tickLegs()
{
#ifdef DEBUG_STEPGENERATOR
	cout << "StepGenerator::tick_legs" << endl;
#endif

	sensorAngles.tick_sensors();

	
	if(leftLeg.isSwitchingSupportMode() && leftLeg.stateIsDoubleSupport()){
		swapSupportLegs();
	}


	 
	 leftLeg.setSteps(swingingStepSource_f, swingingStep_f,supportStep_f);
    rightLeg.setSteps(swingingStepSource_f, swingingStep_f,supportStep_f);



	com_f = (if_Transform*com_i);

/*	std::cout<<"com_i"<<com_i<<std::endl;*/
	
	const float body_rot_angle_fc = leftLeg.getFootRotation()/2;

	
	fc_Transform = (Kinematics::rotation3D(Kinematics::Z_AXIS,
		body_rot_angle_fc))*(Kinematics::translation3D(-com_f(0),-com_f(1)));

	

	std::tr1::shared_ptr<Step> leftStep_f,rightStep_f;

	
	if (supportStep_f->foot == LEFT_FOOT){
		leftStep_f = supportStep_f;
		rightStep_f = swingingStep_f;
	}
	else{
		rightStep_f = supportStep_f;
		leftStep_f = swingingStep_f;
	}

	

	LegJointStiffTuple left  = leftLeg.tick(leftStep_f,swingingStepSource_f,
		swingingStep_f,fc_Transform);
	LegJointStiffTuple right = rightLeg.tick(rightStep_f,swingingStepSource_f,
		swingingStep_f,fc_Transform);


	if(supportStep_f->foot == LEFT_FOOT){
		updateOdometry(leftLeg.getOdoUpdate());
	}else{
		updateOdometry(rightLeg.getOdoUpdate());
	}

	
	if(supportStep_s->type == END_STEP && swingingStep_s->type == END_STEP
		&& lastStep_s->type == END_STEP &&
		
		x == 0.0f && y == 0.0f && theta == 0.0f) {
			done = true;
			/*throw;*/
	}

#ifdef DEBUG_STEPGENERATOR
	cout << "StepGenerator::tick_legs DONE" << endl;
#endif

	return WalkLegsTuple(left,right);

}



void StepGenerator::updateOdometry(const vector<float> &deltaOdo){
	const Eigen::Matrix3f odoUpdate = (Kinematics::translation3D(deltaOdo[0],
		deltaOdo[1]))*
		(Kinematics::rotation3D(Kinematics::Z_AXIS,
		-deltaOdo[2]));
	const Eigen::Matrix3f new_cc_Transform  = (cc_Transform*odoUpdate);
	cc_Transform = new_cc_Transform;

}

void StepGenerator::swapSupportLegs()
{
	if (currentZMPDSteps.size() +  futureSteps.size() <
		MIN_NUM_ENQUEUED_STEPS)
		throw "Insufficient steps";

	lastStep_s = *currentZMPDSteps.begin();
	currentZMPDSteps.pop_front();
	swingingStep_s  = *(++currentZMPDSteps.begin());
	supportStep_s   =  *(currentZMPDSteps.begin());
	supportFoot = (supportStep_s->foot == LEFT_FOOT ?
LEFT_SUPPORT : RIGHT_SUPPORT);



	Eigen::Matrix3f stepTransform = get_fprime_f(supportStep_s);
	if_Transform =stepTransform*if_Transform;


	const  Eigen::Vector3f origin(0,0,1);
	const  Eigen::Vector3f supp_pos_f = origin;


	Eigen::Vector3f swing_src_f = (stepTransform*origin);


	const Eigen::Matrix3f swing_reverse_trans =
		get_f_fprime(swingingStep_s);

	const Eigen::Vector3f swing_pos_f = (swing_reverse_trans*
		origin);
	
	float swing_dest_angle = -safe_asin(swing_reverse_trans(1,0));

	float swing_src_angle = -safe_asin(stepTransform(1,0));




	supportStep_f =
		shared_ptr<Step>(new Step(supp_pos_f(0),supp_pos_f(1),
		0.0f,*supportStep_s));
	swingingStep_f =
		shared_ptr<Step>(new Step(swing_pos_f(0),swing_pos_f(1),
		swing_dest_angle,*swingingStep_s));
	swingingStepSource_f  =
		shared_ptr<Step>(new Step(swing_src_f(0),swing_src_f(1),
		swing_src_angle,*lastStep_s));
	
}

const Eigen::Matrix3f StepGenerator::get_fprime_f( const std::tr1::shared_ptr<Step> step )
{

	const float leg_sign = (step->foot == LEFT_FOOT ? 1.0f : -1.0f);

	const float x = step->x;
	const float y = step->y;
	const float theta = step->theta;

	Eigen::Matrix3f trans_fprime_s =
		
		Kinematics::translation3D(0,-leg_sign*HIP_OFFSET_Y);

	Eigen::Matrix3f trans_s_f =
		(Kinematics::rotation3D(Kinematics::Z_AXIS,-theta))*(
		Kinematics::translation3D(-x,-y));
	return (trans_s_f*trans_fprime_s);
}

const Eigen::Matrix3f StepGenerator::get_f_fprime( const std::tr1::shared_ptr<Step> step )
{
	const float leg_sign = (step->foot == LEFT_FOOT ? 1.0f : -1.0f);

	const float x = step->x;
	const float y = step->y;
	const float theta = step->theta;

	Eigen::Matrix3f  trans_fprime_s =
		Kinematics::translation3D(0,leg_sign*HIP_OFFSET_Y);

	Eigen::Matrix3f  trans_s_f =
		(Kinematics::translation3D(x,y))*
		(Kinematics::rotation3D(Kinematics::Z_AXIS,theta));
	return trans_fprime_s*trans_s_f;

}

WalkArmsTuple StepGenerator::tick_arms()
{
	return WalkArmsTuple(leftArm.tick(supportStep_f),
		rightArm.tick(supportStep_f));
}
