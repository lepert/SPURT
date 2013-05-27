#include "WBSensorsTranslator.h"
#include <webots/accelerometer.h>
#include <webots/robot.h>
#include <webots/gyro.h>
#define COMPUTE_WEBOTS_ANGLE


WBSensorsTranslator::~WBSensorsTranslator(void)
{
}

WBSensorsTranslator::WBSensorsTranslator(std::tr1::shared_ptr<GeneralSensors> s )
	:translator(s),
	angleEKF()
{
	prevAngleX = 0;
	prevAngleY = 0;
	 acc = wb_robot_get_device("accelerometer");
	wb_accelerometer_enable (acc, 20);

	 gyro = wb_robot_get_device("gyro");
	wb_gyro_enable (gyro, 20);
	  
}

void WBSensorsTranslator::sendMotionSensors()
{
	


	const double *acc_values = wb_accelerometer_get_values (acc);
	    const double *gyro_values = wb_gyro_get_values (gyro);
	

		//webots units are already in m/ss, but the signs may be wack...

		const float accX = -static_cast<float>(acc_values[0]);
		const float accY = -static_cast<float>(acc_values[1]);
		const float accZ = -static_cast<float>(acc_values[2]);

		const float gyroY = static_cast<float>(gyro_values[0]);
		const float gyroX = static_cast<float>(gyro_values[1]);

	//	std::cout<<"accX "<<accX<<"accX "<<accY<<"accX "<<accZ<<"accX "<<gyroY<<"accX "<<gyroX;
	
	/*

	показания акселерометра идентифицируют
	падение если значения А>1 или <-1

	*/
		

		const boost::tuple<const float, const float> angles =
			angleWrapper(accX,
			accY,
			gyroX,
			gyroY);


		const float angleX = angles.get<0>();
		const float angleY = angles.get<1>();
/*

		 std::cout<<"angleX: "<<angleX*MMath::TO_DEG<< std::endl;
		 std::cout<<"angleY: "<<angleY*MMath::TO_DEG<< std::endl;*/
		Inertial wbInertial= Inertial(accX,accY,accZ,gyroX,gyroY,angleX,angleY);

		//Put all the structs, etc together, and send them to sensors


		translator->setMotionSensors(wbInertial);

}

/*фильтр калмана для инерциальных данных*/

const boost::tuple<const float, const float> 
	WBSensorsTranslator::angleWrapper( const float accX,
	const float accY, 
	const float gyroX,
	const float gyroY )
{


#ifdef COMPUTE_WEBOTS_ANGLE


	/*перевод показаний для фильтра*/
    const float ratioX = -accX/GRAVITY_mss;
    const float ratioY = -accY/GRAVITY_mss;

	

    float accAngleX = std::asin(ratioX);
    float accAngleY = std::asin(ratioY);
	
	/*определяем падение*/
	
    if (ratioX >= 1){
      accAngleX = MMath::M_PI_FLOAT/2;
      prevAngleX = MMath::M_PI_FLOAT/2;
    }
    else
      if (ratioX <= -1){
	accAngleX = -MMath::M_PI_FLOAT/2;
	prevAngleX = -MMath::M_PI_FLOAT/2;
      }

    if (ratioY >= 1){
      accAngleY = MMath::M_PI_FLOAT/2;
      prevAngleY = MMath::M_PI_FLOAT/2;
    }
    else
      if (ratioY <= -1){
	accAngleY = -MMath::M_PI_FLOAT/2;
	prevAngleY = -MMath::M_PI_FLOAT/2;
      }



angleEKF.update(accAngleX, accAngleY);


if (accAngleX <= .002 && accAngleX >= -.002)
	prevAngleX = 0;
if (accAngleY <= .002 && accAngleY >= -.002)
	prevAngleY = 0;

angleEKF.update(prevAngleX + gyroX*0.02, prevAngleY + gyroY*0.02);


const float angleX = angleEKF.getAngleX();
const float angleY = angleEKF.getAngleY();

prevAngleX += gyroX*0.02f;
prevAngleY += gyroY*0.02f;


//cout<<"angleX: "<<angleX*TO_DEG<<endl<<"angleY: "<<angleY*TO_DEG<<"\n";

//HACK!!!! TODO compute angleX and angleY better (filter?)
//Currently when the gravity accell is in one direction,
//we use that to consider that the robot is rotated along the other axis
//     const float angleX = accY/GRAVITY_mss * M_PI_FLOAT;
//     const float angleY = -accX/GRAVITY_mss * M_PI_FLOAT;
//better approximation, for now


return boost::tuple<const float, const float>(angleX, angleY);
#else
return boost::tuple<const float, const float>(0.0f, 0.0f);	
#endif

}

