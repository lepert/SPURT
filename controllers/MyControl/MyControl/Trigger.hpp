#pragma once
#include "MyPoint.hpp"
class Trigger
{
public:
	Trigger(void);
	~Trigger(void);
	//���������� ���������
	bool GeoLocation3DTrigger(MyPoint Object,MyPoint Target, float Radius);
	bool GeoLocation2DTrigger(MyPoint Object,MyPoint Target, float Radius);



	//������ ���������� �������(���������)
	template <typename T>
	bool EqualStateTrigger( T &ObjectState, T& TargetState);
	//��������������� ���������� �������
	template <typename T>
	bool ApproximateStateTrigger( T &ObjectState, T& TargetState, T& Dif);


	template <typename T>
	bool LessThanTrigger( T &ObjectState, T& LimitState);

};
