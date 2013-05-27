#include "CoordFrame.h"


const Eigen::Matrix3f Kinematics::rotation3D( const Axis axis, const float angle )
{
	Eigen::Matrix3f rot=Eigen::Matrix3f::Identity();
	
	/*микрооптимизация*/
	if (angle == 0.0) { 
		return rot;
	}
	const float sinAngle = std::sin(angle);
	const float cosAngle = std::cos(angle);

	switch(axis) {
	case Kinematics::Z_AXIS:
		rot(Kinematics::X_AXIS,Kinematics:: X_AXIS) =  cosAngle;
		rot(Kinematics::X_AXIS,Kinematics:: Y_AXIS) = -sinAngle;
		rot(Kinematics::Y_AXIS, Kinematics::X_AXIS) =  sinAngle;
		rot(Kinematics::Y_AXIS, Kinematics::Y_AXIS) =  cosAngle;
		break;
	default:
		break;
	}
	return rot;


}

const Eigen::Matrix3f Kinematics::translation3D( const float dx, const float dy )
{
	Eigen::Matrix3f trans =Eigen::Matrix3f::Identity();
	trans(X_AXIS, Z_AXIS) = dx;
	trans(Y_AXIS, Z_AXIS) = dy;
	return trans;
}
