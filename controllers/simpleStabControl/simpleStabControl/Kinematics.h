#ifndef KINEMATICS_H
#define KINEMATICS_H
#include "MMath.h"
#include <string>
using namespace MMath;



namespace Kinematics
{
	enum SupportFoot {
		LEFT_SUPPORT = 0,
		RIGHT_SUPPORT
	};

	enum Axis {
		X_AXIS = 0,
		Y_AXIS,
		Z_AXIS,
		W_AXIS
	};

	/// Joint Name constants ///
	enum JointNames {
		HEAD_YAW = 0,
		HEAD_PITCH,
		// LARM,
		L_SHOULDER_PITCH,
		L_SHOULDER_ROLL,
		L_ELBOW_YAW,
		L_ELBOW_ROLL,
		// LLEG,
		L_HIP_YAW_PITCH,
		L_HIP_ROLL,
		L_HIP_PITCH,
		L_KNEE_PITCH,
		L_ANKLE_PITCH,
		L_ANKLE_ROLL,
		// RLEG,
		R_HIP_YAW_PITCH,
		R_HIP_ROLL,
		R_HIP_PITCH,
		R_KNEE_PITCH,
		R_ANKLE_PITCH,
		R_ANKLE_ROLL,
		// RARM,
		R_SHOULDER_PITCH,
		R_SHOULDER_ROLL,
		R_ELBOW_YAW,
		R_ELBOW_ROLL
	};

	static const unsigned int HEAD_JOINTS = 2;
	static const unsigned int ARM_JOINTS = 4;
	static const unsigned int TIME_STEP_GLOBAL = 40;
	static const unsigned int LEG_JOINTS = 6;
	static const unsigned int NUM_JOINTS = HEAD_JOINTS + ARM_JOINTS*2 +LEG_JOINTS*2;
	
	static const std::string JOINT_STRINGS[NUM_JOINTS] =
	{ "HeadYaw",
	"HeadPitch",
	"LShoulderPitch",
	"LShoulderRoll",
	"LElbowYaw",
	"LElbowRoll",
	"LHipYawPitch",/*6*/
	"LHipRoll",/*7*/
	"LHipPitch",/*8*/
	"LKneePitch",/*9*/
	"LAnklePitch",/*10*/
	"LAnkleRoll",
	"RHipYawPitch",
	"RHipRoll",
	"RHipPitch",
	"RKneePitch",
	"RAnklePitch",
	"RAnkleRoll",
	"RShoulderPitch",
	"RShoulderRoll",
	"RElbowYaw",
	"RElbowRoll"};

	enum ChainID {
		HEAD_CHAIN = 0,
		LARM_CHAIN,
		LLEG_CHAIN,
		RLEG_CHAIN,
		RARM_CHAIN,
		LANKLE_CHAIN, // до лодыжки
		RANKLE_CHAIN  // (то же)
	};

	/**@breif структура описвани€ конечностей 
	@detailed структура содержит логические св€зи конечностей
	напр: создание структуры конечности головы 
	HEAD_SERVOS=new Limbs(HEAD_CHAIN,HEAD_JOINTS,JOINT_STRINGS);
	-хранить идентификатор цепочки enum ChainID
	-хранит количество сочленений
	-назвыни€ WEBOTS world сочлинений
	@paramIn 
	-const ChainID _id ид цепочки конечности
	-const int _numJoints  количество сочленений
	-const std::string *_sensorPtr указатель на первое им€
	дл€ данной цепочки в списке JOINT_STRINGS
	«ависит от ChainID JOINT_STRINGS X_JOINTS X- ид конечности
	*/
	struct Limbs{
	
		ChainID id;
		int numberOfJoints;
		std::string *servosLimb;

	public:Limbs(const ChainID _id, const int _numJoints,const std::string *_sensorPtr)
		   {
			   id=_id;
			   numberOfJoints=_numJoints;
			   servosLimb=new std::string [_numJoints];
			   getNamesServoStr(_sensorPtr,servosLimb,_numJoints);
		   }

		   inline void getNamesServoStr(const std::string * sensorStrPtr, std::string *servosLimbPtr, const int numJ)
		   {

				for (int i=0;i<numJ;i++)
				{
					servosLimbPtr[i]=*(sensorStrPtr+i);
				}
		   }


	};

	struct IKResult {
		float angles[6];
		bool successfulIK;
	};

	static const Limbs* HEAD_SERVOS=new Limbs(HEAD_CHAIN,HEAD_JOINTS,JOINT_STRINGS);
	static const Limbs* LARM_SERVOS=new Limbs(LARM_CHAIN,ARM_JOINTS,JOINT_STRINGS+HEAD_JOINTS);
	static const Limbs* LLEG_SERVOS=new Limbs(LLEG_CHAIN,LEG_JOINTS,JOINT_STRINGS+HEAD_JOINTS+ARM_JOINTS);
	static const Limbs* RLEG_SERVOS=new Limbs(LARM_CHAIN,LEG_JOINTS,JOINT_STRINGS+HEAD_JOINTS+ARM_JOINTS+LEG_JOINTS);
	static const Limbs* RARM_SERVOS=new Limbs(RARM_CHAIN,ARM_JOINTS,JOINT_STRINGS+HEAD_JOINTS+ARM_JOINTS+LEG_JOINTS*2);
	




	/**********  gпараметры тела     ***********/

	static const float SHOULDER_OFFSET_Y = 98.0f;
	static const float UPPER_ARM_LENGTH = 90.0f;
	static const float LOWER_ARM_LENGTH = 145.0f;
	static const float SHOULDER_OFFSET_Z = 100.0f;
	static const float THIGH_LENGTH = 100.0f;
	static const float TIBIA_LENGTH = 100.0f;
	static const float NECK_OFFSET_Z = 126.5f;
	static const float HIP_OFFSET_Y = 50.0f;
	static const float HIP_OFFSET_Z = 85.0f;
	static const float FOOT_HEIGHT = 46.0f;

	//                                  (alpha,  a ,  theta ,   d  )
	const float HEAD_MDH_PARAMS[2][4] = {{0.0f , 0.0f,  0.0f , 0.0f},
	{-M_PI_FLOAT/2, 0.0f, -M_PI_FLOAT/2 , 0.0f}};

	const float LEFT_ARM_MDH_PARAMS[4][4] = {{-M_PI_FLOAT/2,0.0f,0.0f,0.0f},
	{ M_PI_FLOAT/2,0.0f,M_PI_FLOAT/2,0.0f},
	{ M_PI_FLOAT/2,0.0f,0.0f,UPPER_ARM_LENGTH},
	{-M_PI_FLOAT/2,0.0f,0.0f,0.0f}};

	const float LEFT_LEG_MDH_PARAMS[6][4] = {{ -3*M_PI_FLOAT/4, 0.0f,  -M_PI_FLOAT/2, 0.0f},
	{ -M_PI_FLOAT/2,   0.0f,   M_PI_FLOAT/4, 0.0f},
	{ M_PI_FLOAT/2,    0.0f,     0.0f, 0.0f},
	{   0.0f,-THIGH_LENGTH,0.0f, 0.0f},
	{   0.0f,-TIBIA_LENGTH,0.0f, 0.0f},
	{-M_PI_FLOAT/2,    0.0f,     0.0f, 0.0f}};

	const float RIGHT_LEG_MDH_PARAMS[6][4]= {{ -M_PI_FLOAT/4,  0.0f,   -M_PI_FLOAT/2, 0.0f},
	{ -M_PI_FLOAT/2,   0.0f,  -M_PI_FLOAT/4, 0.0f},
	{  M_PI_FLOAT/2,    0.0f,    0.0f, 0.0f},
	{ 0.0f,-THIGH_LENGTH,0.0f, 0.0f},
	{0.0f,-TIBIA_LENGTH,0.0f,0.0f},
	{-M_PI_FLOAT/2,0.0f,0.0f,0.0f}};

	const float RIGHT_ARM_MDH_PARAMS[4][4] = {{-M_PI_FLOAT/2, 0.0f,0.0f,0.0f},
	{ M_PI_FLOAT/2, 0.0f,M_PI_FLOAT/2,0.0f},
	{ M_PI_FLOAT/2, 0.0f,0.0f,UPPER_ARM_LENGTH},
	{-M_PI_FLOAT/2, 0.0f,0.0f,0.0f}};
}
#endif //KINEMATICS_H    