#ifndef GENERAL_SENSORS_H
#define GENERAL_SENSORS_H
#include <vector>
#include <iostream>
#include <stdio.h>

struct FSR {
	FSR(const float fl, const float fr,
		const float rl, const float rr)
		: frontLeft(fl), frontRight(fr), rearLeft(rl), rearRight(rr) { }

	float frontLeft;
	float frontRight;
	float rearLeft;
	float rearRight;
};
struct Inertial {
	Inertial(const float _accX, const float _accY, const float _accZ,
		const float _gyrX, const float _gyrY,
		const float _angleX, const float _angleY)
		: accX(_accX), accY(_accY), accZ(_accZ),
		gyrX(_gyrX), gyrY(_gyrY), angleX(_angleX), angleY(_angleY) { }

	float accX;
	float accY;
	float accZ;
	float gyrX;
	float gyrY;
	float angleX;
	float angleY;
};

class GeneralSensors
{
public:

	GeneralSensors(void);
	~GeneralSensors(void);


	const Inertial getInertial () const;
	void setMotionSensors(		const Inertial &_inertial);
private:
	
	Inertial inertial;

};

#endif//GENERAL_SENSORS_H