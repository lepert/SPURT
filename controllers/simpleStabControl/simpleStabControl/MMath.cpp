#include "MMath.h"


const Eigen::Matrix4f MMath::get6fTransform( const float x, const float y, const float z, const float wx, const float wy, const float wz )
{

	float cwx,cwy,cwz,
		swx,swy,swz;
	sincosf(wx,&swx,&cwx);
	sincosf(wy,&swy,&cwy);
	sincosf(wz,&swz,&cwz);

	Eigen::Matrix4f resultMatrix=Eigen::Matrix4f::Identity();

	//строка1
	resultMatrix(0,0) =cwy*cwz;
	resultMatrix(0,1) =cwz*swx*swy-cwx*swz;
	resultMatrix(0,2) =cwx*cwz*swy+swx*swz;
	resultMatrix(0,3) =x;
	//строка2
	resultMatrix(1,0) =cwy*swz;
	resultMatrix(1,1) =cwx*cwz+swx*swy*swz;
	resultMatrix(1,2) =-cwz*swx+cwx*swy*swz;
	resultMatrix(1,3) =y;
	//строка3
	resultMatrix(2,0) =-swy;
	resultMatrix(2,1) =cwy*swx;
	resultMatrix(2,2) =cwx*cwy;
	resultMatrix(2,3) =z;

	return resultMatrix;


}

const Eigen::Matrix4f MMath::invertHomogenous( const Eigen::Matrix4f &sourse)
{

	Eigen::Matrix3f Rt = sourse.block(0,0,3,3);
	Rt.transposeInPlace();
	 const Eigen::Vector3f tempV( sourse(0,3),sourse(1,3),sourse(2,3));
	 const  Eigen::Vector3f Rtd = -(Rt*tempV);
	// std::cout<< "Rtd"<< Rtd[0]<<"-"<<Rtd[1]<<"-"<<Rtd[2]<<" ";
	 Eigen::Matrix4f res = Eigen::Matrix4f::Identity();
	 res.block(0,0,3,3)= Rt;
	 res(0,3) = Rtd(0);
	 res(1,3) = Rtd(1);
	 res(2,3) =Rtd(2);

 	 return res;
}


const float MMath::safe_atan2(const float y, const float x)
{
	if (x == 0.0f) {
		if ( y > 0.0f) {
			return MMath::M_PI_FLOAT / 2.0f;
		} else {
			return -MMath::M_PI_FLOAT / 2.0f;
		}
	}
	return atan2(y,x);
}

const float MMath::clip(const float value, const float minMax){
	return clip(value,-minMax,minMax);
}

const float MMath::clip(const float value, const float minValue,
	const float maxValue) {
		if (value > maxValue)
			return maxValue;
		else if (value < minValue)
			return minValue;
 		else if(!(value)||value==0)
 			return 0.0f;
		else
			return value;
}

const float MMath::cliptoZero(const float value, const float minValue)
{
		if (value < minValue)
			return 0;
		else if(!(value)||value==0)
			return 0.0f;
		else
			return value;
}



 const float MMath::safe_asin( const float input )
 {
	 return std::asin(clip(input,1.0f));
 }

 const float MMath::cycloidx( const float theta )
 {
	
		 return theta - std::sin(theta);
	

 }

 const float MMath::cycloidy( const float theta )
 {
	 return 1.0f - std::cos(theta);
 }
