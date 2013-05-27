#include "ForwardKinematics.h"

// *  TTL --  THIGH_LENGTH, TIBIA_LENTH, legLength
// 	*         расстояние лодыжка-бедро


const Vector3d Kinematics::forwardKinematics( const ChainID id, const std::vector<float> &angles )
{

	float x=0.0f,y=0.0f,z=0.0f;
	if(id == LLEG_CHAIN || id == RLEG_CHAIN||
		id == LANKLE_CHAIN || id == RANKLE_CHAIN)
	{
		const float HYP = angles[0];
		const float HR = angles[1];
		const float HP = angles[2];
		const float KP = angles[3];
		const float AP = angles[4];
		const float AR = angles[5];

		float sinHYP,cosHYP,sinHR,cosHR,sinHP,cosHP,sinKP,cosKP,sinAP,cosAP,sinAR,cosAR;
		
		sincosf(HYP,&sinHYP,&cosHYP);
		sincosf(HR,&sinHR,&cosHR);
		sincosf(HP,&sinHP,&cosHP);
		sincosf(KP,&sinKP,&cosKP);
		sincosf(AP,&sinAP,&cosAP);
		sincosf(AR,&sinAR,&cosAR);

		const float cosHRPlusPiFourth = std::cos(HR+M_PI_FLOAT*0.25f);
		const float cosHRMinusPiFourth = std::cos(HR-M_PI_FLOAT*0.25f);
		const float sinHRPlusPiFourth = std::sin(HR+M_PI_FLOAT*0.25f);
		const float sinHRMinusPiFourth = std::sin(HR-M_PI_FLOAT*0.25f);
		const float sqrt2 = std::sqrt(2.0f);

		switch(id){
		case LLEG_CHAIN:
			x = -THIGH_LENGTH*(cosHYP*sinHP+cosHP*cosHRPlusPiFourth*sinHYP)-TIBIA_LENGTH*(cosKP*(cosHYP*sinHP+cosHP*cosHRPlusPiFourth*sinHYP)+(cosHP*cosHYP-cosHRPlusPiFourth*sinHP*sinHYP)*sinKP)-FOOT_HEIGHT*(cosAR*(sinAP*(cosKP*(cosHP*cosHYP-cosHRPlusPiFourth*sinHP*sinHYP)-(cosHYP*sinHP+cosHP*cosHRPlusPiFourth*sinHYP)*sinKP)+cosAP*(cosKP*(cosHYP*sinHP+cosHP*cosHRPlusPiFourth*sinHYP)+(cosHP*cosHYP-cosHRPlusPiFourth*sinHP*sinHYP)*sinKP))-sinAR*sinHYP*sinHRPlusPiFourth);
			y = HIP_OFFSET_Y-THIGH_LENGTH*(-(sinHP*sinHYP)/sqrt2+cosHP*((cosHYP*cosHRPlusPiFourth)/sqrt2-sinHRPlusPiFourth/sqrt2))-TIBIA_LENGTH*(cosKP*(-(sinHP*sinHYP)/sqrt2+cosHP*((cosHYP*cosHRPlusPiFourth)/sqrt2-sinHRPlusPiFourth/sqrt2))+sinKP*(-(cosHP*sinHYP)/sqrt2-sinHP*((cosHYP*cosHRPlusPiFourth)/sqrt2-sinHRPlusPiFourth/sqrt2)))-FOOT_HEIGHT*(-sinAR*(cosHRPlusPiFourth/sqrt2+(cosHYP*sinHRPlusPiFourth)/sqrt2)+cosAR*(sinAP*(-sinKP*(-(sinHP*sinHYP)/sqrt2+cosHP*((cosHYP*cosHRPlusPiFourth)/sqrt2-sinHRPlusPiFourth/sqrt2))+cosKP*(-(cosHP*sinHYP)/sqrt2-sinHP*((cosHYP*cosHRPlusPiFourth)/sqrt2-sinHRPlusPiFourth/sqrt2)))+cosAP*(cosKP*(-(sinHP*sinHYP)/sqrt2+cosHP*((cosHYP*cosHRPlusPiFourth)/sqrt2-sinHRPlusPiFourth/sqrt2))+sinKP*(-(cosHP*sinHYP)/sqrt2-sinHP*((cosHYP*cosHRPlusPiFourth)/sqrt2-sinHRPlusPiFourth/sqrt2)))));
			z = -HIP_OFFSET_Z-THIGH_LENGTH*(-(sinHP*sinHYP)/sqrt2+cosHP*((cosHYP*cosHRPlusPiFourth)/sqrt2+sinHRPlusPiFourth/sqrt2))-TIBIA_LENGTH*(cosKP*(-(sinHP*sinHYP)/sqrt2+cosHP*((cosHYP*cosHRPlusPiFourth)/sqrt2+sinHRPlusPiFourth/sqrt2))+sinKP*(-(cosHP*sinHYP)/sqrt2-sinHP*((cosHYP*cosHRPlusPiFourth)/sqrt2+sinHRPlusPiFourth/sqrt2)))-FOOT_HEIGHT*(-sinAR*(-cosHRPlusPiFourth/sqrt2+(cosHYP*sinHRPlusPiFourth)/sqrt2)+cosAR*(sinAP*(-sinKP*(-(sinHP*sinHYP)/sqrt2+cosHP*((cosHYP*cosHRPlusPiFourth)/sqrt2+sinHRPlusPiFourth/sqrt2))+cosKP*(-(cosHP*sinHYP)/sqrt2-sinHP*((cosHYP*cosHRPlusPiFourth)/sqrt2+sinHRPlusPiFourth/sqrt2)))+cosAP*(cosKP*(-(sinHP*sinHYP)/sqrt2+cosHP*((cosHYP*cosHRPlusPiFourth)/sqrt2+sinHRPlusPiFourth/sqrt2))+sinKP*(-(cosHP*sinHYP)/sqrt2-sinHP*((cosHYP*cosHRPlusPiFourth)/sqrt2+sinHRPlusPiFourth/sqrt2)))));
			break;
		case LANKLE_CHAIN:
			x = -THIGH_LENGTH*(cosHYP*sinHP+cosHP*cosHRPlusPiFourth*sinHYP)-TIBIA_LENGTH*(cosKP*(cosHYP*sinHP+cosHP*cosHRPlusPiFourth*sinHYP)+(cosHP*cosHYP-cosHRPlusPiFourth*sinHP*sinHYP)*sinKP);
			y = HIP_OFFSET_Y-THIGH_LENGTH*(-(sinHP*sinHYP)/sqrt2+cosHP*((cosHYP*cosHRPlusPiFourth)/sqrt2-sinHRPlusPiFourth/sqrt2))-TIBIA_LENGTH*(cosKP*(-(sinHP*sinHYP)/sqrt2+cosHP*((cosHYP*cosHRPlusPiFourth)/sqrt2-sinHRPlusPiFourth/sqrt2))+sinKP*(-(cosHP*sinHYP)/sqrt2-sinHP*((cosHYP*cosHRPlusPiFourth)/sqrt2-sinHRPlusPiFourth/sqrt2)));
			z = -HIP_OFFSET_Z-THIGH_LENGTH*(-(sinHP*sinHYP)/sqrt2+cosHP*((cosHYP*cosHRPlusPiFourth)/sqrt2+sinHRPlusPiFourth/sqrt2))-TIBIA_LENGTH*(cosKP*(-(sinHP*sinHYP)/sqrt2+cosHP*((cosHYP*cosHRPlusPiFourth)/sqrt2+sinHRPlusPiFourth/sqrt2))+sinKP*(-(cosHP*sinHYP)/sqrt2-sinHP*((cosHYP*cosHRPlusPiFourth)/sqrt2+sinHRPlusPiFourth/sqrt2)));
			break;
		case RLEG_CHAIN:
			x = -THIGH_LENGTH*(cosHYP*sinHP+cosHP*cosHRMinusPiFourth*sinHYP)-TIBIA_LENGTH*(cosKP*(cosHYP*sinHP+cosHP*cosHRMinusPiFourth*sinHYP)+(cosHP*cosHYP-cosHRMinusPiFourth*sinHP*sinHYP)*sinKP)-FOOT_HEIGHT*(cosAR*(sinAP*(cosKP*(cosHP*cosHYP-cosHRMinusPiFourth*sinHP*sinHYP)-(cosHYP*sinHP+cosHP*cosHRMinusPiFourth*sinHYP)*sinKP)+cosAP*(cosKP*(cosHYP*sinHP+cosHP*cosHRMinusPiFourth*sinHYP)+(cosHP*cosHYP-cosHRMinusPiFourth*sinHP*sinHYP)*sinKP))-sinAR*sinHYP*sinHRMinusPiFourth);
			y = -HIP_OFFSET_Y-THIGH_LENGTH*((sinHP*sinHYP)/sqrt2+cosHP*(-(cosHYP*cosHRMinusPiFourth)/sqrt2-sinHRMinusPiFourth/sqrt2))-TIBIA_LENGTH*(cosKP*((sinHP*sinHYP)/sqrt2+cosHP*(-(cosHYP*cosHRMinusPiFourth)/sqrt2-sinHRMinusPiFourth/sqrt2))+sinKP*((cosHP*sinHYP)/sqrt2-sinHP*(-(cosHYP*cosHRMinusPiFourth)/sqrt2-sinHRMinusPiFourth/sqrt2)))-FOOT_HEIGHT*(-sinAR*(cosHRMinusPiFourth/sqrt2-(cosHYP*sinHRMinusPiFourth)/sqrt2)+cosAR*(sinAP*(-sinKP*((sinHP*sinHYP)/sqrt2+cosHP*(-(cosHYP*cosHRMinusPiFourth)/sqrt2-sinHRMinusPiFourth/sqrt2))+cosKP*((cosHP*sinHYP)/sqrt2-sinHP*(-(cosHYP*cosHRMinusPiFourth)/sqrt2-sinHRMinusPiFourth/sqrt2)))+cosAP*(cosKP*((sinHP*sinHYP)/sqrt2+cosHP*(-(cosHYP*cosHRMinusPiFourth)/sqrt2-sinHRMinusPiFourth/sqrt2))+sinKP*((cosHP*sinHYP)/sqrt2-sinHP*(-(cosHYP*cosHRMinusPiFourth)/sqrt2-sinHRMinusPiFourth/sqrt2)))));
			z = -HIP_OFFSET_Z-THIGH_LENGTH*(-(sinHP*sinHYP)/sqrt2+cosHP*((cosHYP*cosHRMinusPiFourth)/sqrt2-sinHRMinusPiFourth/sqrt2))-TIBIA_LENGTH*(cosKP*(-(sinHP*sinHYP)/sqrt2+cosHP*((cosHYP*cosHRMinusPiFourth)/sqrt2-sinHRMinusPiFourth/sqrt2))+sinKP*(-(cosHP*sinHYP)/sqrt2-sinHP*((cosHYP*cosHRMinusPiFourth)/sqrt2-sinHRMinusPiFourth/sqrt2)))-FOOT_HEIGHT*(-sinAR*(cosHRMinusPiFourth/sqrt2+(cosHYP*sinHRMinusPiFourth)/sqrt2)+cosAR*(sinAP*(-sinKP*(-(sinHP*sinHYP)/sqrt2+cosHP*((cosHYP*cosHRMinusPiFourth)/sqrt2-sinHRMinusPiFourth/sqrt2))+cosKP*(-(cosHP*sinHYP)/sqrt2-sinHP*((cosHYP*cosHRMinusPiFourth)/sqrt2-sinHRMinusPiFourth/sqrt2)))+cosAP*(cosKP*(-(sinHP*sinHYP)/sqrt2+cosHP*((cosHYP*cosHRMinusPiFourth)/sqrt2-sinHRMinusPiFourth/sqrt2))+sinKP*(-(cosHP*sinHYP)/sqrt2-sinHP*((cosHYP*cosHRMinusPiFourth)/sqrt2-sinHRMinusPiFourth/sqrt2)))));
			break;
		case RANKLE_CHAIN:
			x = -THIGH_LENGTH*(cosHYP*sinHP+cosHP*cosHRMinusPiFourth*sinHYP)-TIBIA_LENGTH*(cosKP*(cosHYP*sinHP+cosHP*cosHRMinusPiFourth*sinHYP)+(cosHP*cosHYP-cosHRMinusPiFourth*sinHP*sinHYP)*sinKP);
			y = -HIP_OFFSET_Y-THIGH_LENGTH*((sinHP*sinHYP)/sqrt2+cosHP*(-(cosHYP*cosHRMinusPiFourth)/sqrt2-sinHRMinusPiFourth/sqrt2))-TIBIA_LENGTH*(cosKP*((sinHP*sinHYP)/sqrt2+cosHP*(-(cosHYP*cosHRMinusPiFourth)/sqrt2-sinHRMinusPiFourth/sqrt2))+sinKP*((cosHP*sinHYP)/sqrt2-sinHP*(-(cosHYP*cosHRMinusPiFourth)/sqrt2-sinHRMinusPiFourth/sqrt2)));
			z = -HIP_OFFSET_Z-THIGH_LENGTH*(-(sinHP*sinHYP)/sqrt2+cosHP*((cosHYP*cosHRMinusPiFourth)/sqrt2-sinHRMinusPiFourth/sqrt2))-TIBIA_LENGTH*(cosKP*(-(sinHP*sinHYP)/sqrt2+cosHP*((cosHYP*cosHRMinusPiFourth)/sqrt2-sinHRMinusPiFourth/sqrt2))+sinKP*(-(cosHP*sinHYP)/sqrt2-sinHP*((cosHYP*cosHRMinusPiFourth)/sqrt2-sinHRMinusPiFourth/sqrt2)));
			break;

	}
	}
	else if( id == LARM_CHAIN || id == RARM_CHAIN ){
		// Variables for arms.
		const float SP = angles[0];
		const float SR = angles[1];
		const float EY = angles[2];
		const float ER = angles[3];

		float sinSP, cosSP, sinSR, cosSR, sinEY, cosEY, sinER, cosER;
		sincosf(SP, &sinSP, &cosSP);
		sincosf(SR, &sinSR, &cosSR);
		sincosf(EY, &sinEY, &cosEY);
		sincosf(ER, &sinER, &cosER);
		switch(id){
		case LARM_CHAIN:
			x = LOWER_ARM_LENGTH*sinER*sinEY*sinSP + cosSP*((UPPER_ARM_LENGTH + LOWER_ARM_LENGTH*cosER)*cosSR - LOWER_ARM_LENGTH*cosEY*sinER*sinSR);
			y = SHOULDER_OFFSET_Y + LOWER_ARM_LENGTH*cosEY*cosSR*sinER + (UPPER_ARM_LENGTH + LOWER_ARM_LENGTH*cosER)*sinSR;
			z = SHOULDER_OFFSET_Z + LOWER_ARM_LENGTH*cosSP*sinER*sinEY - (UPPER_ARM_LENGTH + LOWER_ARM_LENGTH*cosER)*cosSR*sinSP + LOWER_ARM_LENGTH*cosEY*sinER*sinSP*sinSR;
			break;
		case RARM_CHAIN:
			x = LOWER_ARM_LENGTH*sinER*sinEY*sinSP + cosSP* ((UPPER_ARM_LENGTH + LOWER_ARM_LENGTH*cosER)*cosSR - LOWER_ARM_LENGTH*cosEY*sinER*sinSR);
			y = - SHOULDER_OFFSET_Y + LOWER_ARM_LENGTH*cosEY*cosSR*sinER + (UPPER_ARM_LENGTH + LOWER_ARM_LENGTH*cosER)*sinSR;
			z = SHOULDER_OFFSET_Z + LOWER_ARM_LENGTH*cosSP*sinER*sinEY - (UPPER_ARM_LENGTH + LOWER_ARM_LENGTH*cosER)*cosSR*sinSP + LOWER_ARM_LENGTH*cosEY*sinER*sinSP*sinSR;
			break;
		case LLEG_CHAIN:
		case RLEG_CHAIN:
		case RANKLE_CHAIN:
		case LANKLE_CHAIN:
		case HEAD_CHAIN:
			throw "Should not be a possible chain id";
		}
	}else if(id == HEAD_CHAIN){
		x = 0.0f;
		y = 0.0f;
		z = NECK_OFFSET_Z;
	}

	Vector3d result(x,y,z);

	return result;
}
