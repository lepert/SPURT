#include "SampleRobot.h"
#include "StepGenerator.h"

SampleRobot::SampleRobot():counter(0),
	 jointDevices(NUM_JOINTS)
{
	//assign and enable the joint devices

	for(unsigned int joint = 0; joint < NUM_JOINTS; joint++){
		const string devName = JOINT_STRINGS[joint];
		jointDevices[joint] = wb_robot_get_device(devName.c_str());
		wb_servo_enable_position (jointDevices[joint], 20);
	}

	 generalSensors=std::tr1::shared_ptr<GeneralSensors>(new GeneralSensors());
	 stepGenerator= std::tr1::shared_ptr<StepGenerator> (new StepGenerator(generalSensors));
	 wbsensorsTranslator=std::tr1::shared_ptr<WBSensorsTranslator>(new WBSensorsTranslator(generalSensors)) ;
}


SampleRobot::~SampleRobot()
{
}

void SampleRobot::run()
{



	
	








	/**код генераци походки */

	// Walk vector:
	//  * x - скорость продольного движения mm/s
	//  * y - скорость поперечного движения(положительно - влево)mm/s
	//  * theta - угол (против часовой)  



	/**изменение скорости*/
	
	if(counter==0)
	{
	stepGenerator->takeSteps(400, 0,	0 ,	100);
	
	}
	counter++;
	

	

	if(counter==900)
	{
		stepGenerator->setSpeed(-400, 0,	0 );

	}	


	if(counter==1200)
	{
		stepGenerator->setSpeed(0, 400,	0 );

	}

	if(counter==1500)
	{
		stepGenerator->setSpeed(0, -400,	0 );

	}

	if(counter==1800)
	{
		stepGenerator->setSpeed(400, 400,	0 );

	}



	//обновить змп
	/*	std::cout<<counter<<std::endl;*/
	 stepGenerator->tick_controller();

// 	 if (wb_robot_get_time ()>40(ms))
	 //ВАЖНО! показания акселерометра должны генерироваться позде 40 мс
		 wbsensorsTranslator->sendMotionSensors();
	 

	 WalkLegsTuple legs_result=	 stepGenerator->tickLegs();

	
	 WalkArmsTuple arms_result = stepGenerator->tick_arms();

	
		 vector<float> lleg_joints = legs_result.get<LEFT_FOOT>().get<JOINT_INDEX>();
		 vector<float> rleg_joints = legs_result.get<RIGHT_FOOT>().get<JOINT_INDEX>();
		 vector<float> lleg_gains = legs_result.get<LEFT_FOOT>().get<STIFF_INDEX>();
		 vector<float> rleg_gains = legs_result.get<RIGHT_FOOT>().get<STIFF_INDEX>();
	
	
		 vector<float> larm_joints = arms_result.get<LEFT_FOOT>().get<JOINT_INDEX>();
		 vector<float> rarm_joints = arms_result.get<RIGHT_FOOT>().get<JOINT_INDEX>();
		 vector<float> larm_gains = arms_result.get<LEFT_FOOT>().get<STIFF_INDEX>();
		 vector<float> rarm_gains = arms_result.get<RIGHT_FOOT>().get<STIFF_INDEX>();


		 for(unsigned int joint = 0;  joint < NUM_JOINTS; joint++)
			 motionValues.push_back(0);

		 for(unsigned int i = 0; i < LEG_JOINTS; i ++)
		 {
			
			
			 motionValues[R_HIP_YAW_PITCH + i] = rleg_joints.at(i);
			 motionValues[L_HIP_YAW_PITCH + i] = lleg_joints.at(i);
			
		 }

	 for(unsigned int i = 0; i <ARM_JOINTS ; i ++)
		 {
	
				 motionValues[L_SHOULDER_PITCH+i]=larm_joints.at(i);
			 motionValues[R_SHOULDER_PITCH+ i]=rarm_joints.at(i);
		 }



		 for(unsigned int joint = 0;  joint < NUM_JOINTS; joint++)
		 { 
			wb_servo_set_position(jointDevices[joint],motionValues[joint]);
		 
		/* std::cout<<" "<<motionValues[joint];*/
		 }
/* std::cout<<endl;* /*/



}

