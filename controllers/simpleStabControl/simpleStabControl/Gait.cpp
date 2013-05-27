#include "Gait.h"
#include <iostream>
#include "string.h"
#include "stdio.h"
using namespace std;

void Gait::setGaitFromArrays(
	const float _stance[GaitDefaults::LEN_STANCE_CONFIG],
	const float _step[GaitDefaults::LEN_STEP_CONFIG],
	const float _zmp[GaitDefaults::LEN_ZMP_CONFIG],
	const float _hack[GaitDefaults::LEN_HACK_CONFIG],
	const float _sensor[GaitDefaults::LEN_SENSOR_CONFIG],
	const float _stiffness[GaitDefaults::LEN_STIFF_CONFIG],
	const float _odo[GaitDefaults::LEN_ODO_CONFIG],
	const float _arm[GaitDefaults::LEN_ARM_CONFIG]){
		memcpy(stance,_stance,GaitDefaults::LEN_STANCE_CONFIG*sizeof(float));
		memcpy(step,_step,GaitDefaults::LEN_STEP_CONFIG*sizeof(float));
		memcpy(zmp,_zmp,GaitDefaults::LEN_ZMP_CONFIG*sizeof(float));
		memcpy(hack,_hack,GaitDefaults::LEN_HACK_CONFIG*sizeof(float));

		memcpy(sensor,_sensor,GaitDefaults::LEN_SENSOR_CONFIG*sizeof(float));
		memcpy(stiffness,_stiffness,GaitDefaults::LEN_STIFF_CONFIG*sizeof(float));
		memcpy(odo,_odo,GaitDefaults::LEN_ODO_CONFIG*sizeof(float));
		memcpy(arm,_arm,GaitDefaults::LEN_ARM_CONFIG*sizeof(float));

}

Gait::Gait(
	const float _stance_config[GaitDefaults::LEN_STANCE_CONFIG],
	const float _step_config[GaitDefaults::LEN_STEP_CONFIG],
	const float _zmp_config[GaitDefaults::LEN_ZMP_CONFIG],
	const float _joint_hack_config[GaitDefaults::LEN_HACK_CONFIG],
	const float _sensor_config[GaitDefaults::LEN_SENSOR_CONFIG],
	const float _stiffness_config[GaitDefaults::LEN_STIFF_CONFIG],
	const float _odo_config[GaitDefaults::LEN_ODO_CONFIG],
	const float _arm_config[GaitDefaults::LEN_ARM_CONFIG])
{
	setGaitFromArrays(_stance_config,
		_step_config,
		_zmp_config,
		_joint_hack_config,
		_sensor_config,
		_stiffness_config,
		_odo_config,
		_arm_config);
	//cout << "from arrays: "<<endl<< toString() <<endl;
}

Gait::~Gait(void)
{
}
