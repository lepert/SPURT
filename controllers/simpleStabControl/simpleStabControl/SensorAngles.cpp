#include "SensorAngles.h"

SensorAngles::SensorAngles(const Gait * _gait,std::tr1::shared_ptr<GeneralSensors> s):
	gait(_gait),
	sensorAngleX(0.0f),sensorAngleY(0.0f),
	springX(gait,SpringSensor::X),
	springY(gait,SpringSensor::Y),
	sensors(s),
	lastSensorAngleX(0.0f),lastSensorAngleY(0.0f) 
{

}

SensorAngles::~SensorAngles()
{

}
void SensorAngles::reset(){
	
	lastSensorAngleX = lastSensorAngleY = 0.0f;
	sensorAngleX=  sensorAngleY = 0.0f;
}

void SensorAngles::tick_sensors()
{

	 if(gait->sensor[GaitDefaults::FEEDBACK_TYPE] == 1.0f)
		 spring_sensor_feedback();
	 else if(gait->sensor[GaitDefaults::FEEDBACK_TYPE] == 0.0f)
		 basic_sensor_feedback();
	 else{
		 spring_sensor_feedback();

	 }
}

void SensorAngles::spring_sensor_feedback()
{
	const Inertial inertial = sensors->getInertial();
	springX.tick_sensor(inertial.angleX);
	springY.tick_sensor(inertial.angleY-gait->stance[GaitDefaults::BODY_ROT_Y]);
	sensorAngleX = springX.getSensorAngle();
	sensorAngleY = springY.getSensorAngle();


}

const boost::tuple<const float, const float> SensorAngles::getAngles( const float scale /*= 1.0f*/ ) const
{
	return boost::tuple<const float, const float> (sensorAngleX*scale,
		sensorAngleY*scale);

}

void SensorAngles::basic_sensor_feedback()
{

	const float MAX_SENSOR_ANGLE_X = gait->sensor[GaitDefaults::MAX_ANGLE_X];
	const float MAX_SENSOR_ANGLE_Y = gait->sensor[GaitDefaults::MAX_ANGLE_Y];

	const float MAX_SENSOR_VEL = gait->sensor[GaitDefaults::MAX_ANGLE_VEL]*
		MOTION_FRAME_LENGTH_S;


	  Inertial inertial = sensors->getInertial();


	  const float desiredSensorAngleX =
		  inertial.angleX*gait->sensor[GaitDefaults::GAMMA_X];
	  const float desiredSensorAngleY =
		  (inertial.angleY-gait->stance[GaitDefaults::BODY_ROT_Y])
		  *gait->sensor[GaitDefaults::GAMMA_X];



	  sensorAngleX =
		  MMath::clip(
		  MMath::clip(desiredSensorAngleX,
		  desiredSensorAngleX - MAX_SENSOR_VEL,
		  desiredSensorAngleX + MAX_SENSOR_VEL),
		  MAX_SENSOR_ANGLE_X);
	  sensorAngleY =
		  MMath::clip(
		  MMath::clip(desiredSensorAngleY,
		  desiredSensorAngleY - MAX_SENSOR_VEL,
		  desiredSensorAngleY + MAX_SENSOR_VEL),
		  MAX_SENSOR_ANGLE_Y);



	  lastSensorAngleX = sensorAngleX;
	  lastSensorAngleY = sensorAngleY;

}
