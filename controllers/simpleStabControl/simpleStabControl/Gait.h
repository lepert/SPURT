
#ifndef GAIT_H
#define GAIT_H
#include "GaitDefaults.h"

class Gait
{
public:
	Gait(const float _stance_config[GaitDefaults::LEN_STANCE_CONFIG],
		const float _step_config[GaitDefaults::LEN_STEP_CONFIG],
		const float _zmp_config[GaitDefaults::LEN_ZMP_CONFIG],
		const float _joint_hack_config[GaitDefaults::LEN_HACK_CONFIG],
		const float _sensor_config[GaitDefaults::LEN_SENSOR_CONFIG],
		const float _stiffness_config[GaitDefaults::LEN_STIFF_CONFIG],
		const float _odo_config[GaitDefaults::LEN_ODO_CONFIG],
		const float _arm_config[GaitDefaults::LEN_ARM_CONFIG]);

	~Gait(void);


public:
	float stance[GaitDefaults::LEN_STANCE_CONFIG],
		step[GaitDefaults::LEN_STEP_CONFIG],
		zmp[GaitDefaults::LEN_ZMP_CONFIG],
		hack[GaitDefaults::LEN_HACK_CONFIG],
		sensor[GaitDefaults::LEN_SENSOR_CONFIG],
		stiffness[GaitDefaults::LEN_STIFF_CONFIG],
		odo[GaitDefaults::LEN_ODO_CONFIG],
		arm[GaitDefaults::LEN_ARM_CONFIG];

	void setGaitFromArrays(const float _stance_config[GaitDefaults::LEN_STANCE_CONFIG],
		const float _step_config[GaitDefaults::LEN_STEP_CONFIG],
		const float _zmp_config[GaitDefaults::LEN_ZMP_CONFIG],
		const float _joint_hack_config[GaitDefaults::LEN_HACK_CONFIG],
		const float _sensor_config[GaitDefaults::LEN_SENSOR_CONFIG],
		const float _stiffness_config[GaitDefaults::LEN_STIFF_CONFIG],
		const float _odo_config[GaitDefaults::LEN_ODO_CONFIG],
		const float _arm_config[GaitDefaults::LEN_ARM_CONFIG]);

};
static const Gait DEFAULT_GAIT = Gait(GaitDefaults::STANCE_DEFAULT,
	GaitDefaults::STEP_DEFAULT,
	GaitDefaults::ZMP_DEFAULT,
	GaitDefaults::HACK_DEFAULT,
	GaitDefaults::SENSOR_DEFAULT,
	GaitDefaults::STIFF_DEFAULT,
	GaitDefaults::ODO_DEFAULT,
	GaitDefaults::ARM_DEFAULT);


#endif


