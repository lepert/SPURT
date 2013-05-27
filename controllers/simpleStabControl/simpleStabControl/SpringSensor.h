#ifndef SPRING_SENSOR_H 
#define SPRING_SENSOR_H
#include "Gait.h"
#include <Eigen/Core>
#include <Eigen/Dense>
#include "Common.h"
#include "MMath.h"
class SpringSensor
{
public:
	 EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	enum SensorAxis{
		X = 0,
		Y
	};

	 
	 const float getSensorAngle();


	SpringSensor(const Gait * _gait, const SensorAxis _axis);
	~SpringSensor();
	void reset();

	
	void tick_sensor(const float sensorAngle);
private:
	void updateMatrices();
	const Gait * gait;
	const SensorAxis axis;
	const unsigned int K_INDEX;
	const unsigned int GAMMA_INDEX;
	const unsigned int MAX_INDEX;

	Eigen::Vector3f x_k;
	Eigen::Matrix3f A;
	Eigen::Vector3f b;
		Eigen::Vector3f c;//вектор -строка


	std::string name;

	float lastSensorAngle;
};

#endif //SPRING_SENSOR_H