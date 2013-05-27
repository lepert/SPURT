#ifndef WB_SENSORS_TRANSLATOR
#define WB_SENSORS_TRANSLATOR
#include <webots/servo.h>
#include <iostream>
#include <stdio.h>
#include <boost/tuple/tuple.hpp>
#include "Common.h"
#include "MMath.h"
#include "GeneralSensors.h"
#include "AngleEKF.h"


class WBSensorsTranslator
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	WBSensorsTranslator(std::tr1::shared_ptr<GeneralSensors> s);
	~WBSensorsTranslator(void);
	void sendMotionSensors();

	    const boost::tuple<const float, const float>
	angleWrapper(const float accX, const float accY,
		const float gyroX,const float gyroY);
private:
	    float prevAngleX, prevAngleY;
		AngleEKF angleEKF;
		std::tr1::shared_ptr<GeneralSensors> translator;
		WbDeviceTag acc ,gyro;
};

#endif //WB_SENSORS_TRANSLATOR