#pragma once

#include <cstdio>
#include <cmath>

class MyPoint
{
public:
	MyPoint(void);
	MyPoint(float nX,float nY,float nZ);
	MyPoint(float nX,float nY);
	~MyPoint(void);
	float distance2DTo(MyPoint *pDPoint);
	float distance3DTo(MyPoint *pDPoint);
	float cX;
	float cY;
	float cZ;
	// вывод на экран координат (x,y)
	void displayPoint2(void);
	//  вывод на экран координат (x,y,z)
	void displayPoint3(void);
};
