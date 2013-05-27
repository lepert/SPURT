#ifndef WALKING_CONST_H
#define WALKING_CONST_H
#include <vector>
#include <boost/shared_ptr.hpp>
#include "Kinematics.h"



enum SupportMode{
	SUPPORTING=0,
	SWINGING,
	DOUBLE_SUPPORT,
	PERSISTENT_DOUBLE_SUPPORT
};


static const float LARM_WALK_ANGLES[Kinematics::ARM_JOINTS] =
{M_PI_FLOAT/2.0f ,.26f,0.0f,0.0f};
static const float RARM_WALK_ANGLES[Kinematics::ARM_JOINTS] =
{M_PI_FLOAT/2.0f,-.26f,0.0f,0.0f};



const float NEW_VECTOR_THRESH_MMS = 0.0f; 
const float NEW_VECTOR_THRESH_RADS = 0.0f;




#endif//WALKING_CONST_H