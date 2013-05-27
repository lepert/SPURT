#include "GeneralSensors.h"


GeneralSensors::GeneralSensors(void):
      inertial(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f)
{
}




GeneralSensors::~GeneralSensors(void)
{
}



void GeneralSensors::setMotionSensors( const Inertial &_inertial )
{


	inertial =_inertial;
	

}

const Inertial GeneralSensors::getInertial() const
{
	const Inertial inert=inertial;
	
	 return inert;
}
