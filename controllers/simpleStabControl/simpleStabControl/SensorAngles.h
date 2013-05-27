#ifndef SENSORANGLESNAO_H
#define SENSORANGLESNAO_H
#include "Gait.h"
#include "SpringSensor.h"
#include "boost/tuple/tuple.hpp"
#include "GeneralSensors.h"

class SensorAngles
{
public:
	 EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	SensorAngles(const Gait * gait,std::tr1::shared_ptr<GeneralSensors> s);
	~SensorAngles();
	void reset();
	   void tick_sensors();
	   const boost::tuple<const float, const float>
		   getAngles(const float scale = 1.0f) const ;
	   void basic_sensor_feedback();

	   //tuple indices
	   enum SensorAxis{
		   X = 0,
		   Y
	   };


private:

std::tr1::shared_ptr<GeneralSensors> sensors;

	const Gait * gait;
	
	void spring_sensor_feedback();

	float sensorAngleX, sensorAngleY;

	SpringSensor springX,springY;


	float lastSensorAngleX,lastSensorAngleY;

};



#endif //SENSORANGLESNAO_H