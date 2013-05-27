
#include "MyPoint.hpp"

MyPoint::MyPoint(void)
: cX(0)
, cY(0)
, cZ(0)
{
}

MyPoint::MyPoint(float nX,float nY,float nZ)
{
	this->cX=nX;
	this->cY=nY;
	this->cZ=nZ;
}

MyPoint::MyPoint(float nX,float nY)
{
	this->cX=nX;
	this->cY=nY;
	this->cZ=0.0f;
}

MyPoint::~MyPoint(void)
{
}


float MyPoint::distance2DTo(MyPoint *pDPoint)
{
	float dX=pDPoint->cX-this->cX;
	float dY=pDPoint->cY-this->cY;
	return sqrt(dX*dX+dY*dY);
}

float MyPoint::distance3DTo(MyPoint *pDPoint)
{
	float dX=pDPoint->cX-this->cX;
	float dY=pDPoint->cY-this->cY;
	float dZ=pDPoint->cZ-this->cZ;
	return sqrt(dX*dX+dY*dY+dZ*dZ);
}

void MyPoint::displayPoint2(void)
{
	/*printf("x:%f\t y:%f\n",this->cX,this->cY);*/
}

void MyPoint::displayPoint3(void)
{
	/*printf("x:%f\t y:%f\t z:%f\n",this->cX,this->cY,this->cZ);*/
}

