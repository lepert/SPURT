#include "Trigger.hpp"
#include "Math.h"
Trigger::Trigger(void)
{
}

Trigger::~Trigger(void)
{
}
bool Trigger::GeoLocation3DTrigger(MyPoint Object,MyPoint Target, float Radius)
{
	float dX,dY,dZ;
	dX=fabs(Object.cX-Target.cX);
	dY=fabs(Object.cY-Target.cY);
	dZ=fabs(Object.cZ-Target.cZ);
	if(dX<=Radius && dY<Radius && dZ<=Radius)
		return true;
	else
		return false;
}

bool Trigger::GeoLocation2DTrigger(MyPoint Object,MyPoint Target, float Radius)
{
	float dX,dY;
	dX=fabs(Object.cX-Target.cX);
	dY=fabs(Object.cY-Target.cY);
	if(dX<=Radius && dY<Radius)
		return true;
	else
		return false;
}

template <typename T>
bool Trigger::EqualStateTrigger( T &ObjectState, T& TargetState)
{
	if(ObjectState==TargetState)
		return true;
}
template <typename T>
bool Trigger::ApproximateStateTrigger( T &ObjectState, T& TargetState, T& Dif)
{
	if(ObjectState-TargetState<=Dif) && (ObjectState-TargetState>=-Dif)
		return true;
	else
		return false;

}

template <typename T>
bool Trigger::LessThanTrigger(T &ObjectState, T& LimitState)
{
	if (ObjectState < LimitState)
		return true;
	else
		return false;
	}