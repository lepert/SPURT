#include "InverseKinematics.h"

/*#define IK_Debug*/

const Kinematics::IKLegResult Kinematics::analyticLegIK( const ChainID chainID,
	const Vector3f &footGoal,
	const Vector3f &footOrientation,
	const Vector3f &bodyGoal,
	const Vector3f &bodyOrientation,
	const float givenHYPAngle )
{


	
	 bool success = true;

	  IKLegResult res;

#ifdef IK_Debug
	cout << "anaIK inputs:"<<endl
		<<"  footGoal: "<<footGoal<<endl
		<<"  footOrientation: "<<footOrientation<<endl
		<<"  bodyGoal: "<<bodyGoal<<endl
		<<"  bodyOrientation: "<<bodyOrientation<<endl
		<<"  HYP Angle: " <<givenHYPAngle<<endl;
#endif



	/*перевод фремов в универсальную
	систему координат*/

	/*трансформация F-O */
	const Matrix4f fOTransformationMatrix=MMath::get6fTransform(footGoal(0),
		footGoal(1),footGoal(2),
		footOrientation(0),
		footOrientation(1),
		footOrientation(2));

	/*трансформация C-O */

	const Matrix4f cOTransformationMatrix=MMath::get6fTransform(bodyGoal(0),
		bodyGoal(1),bodyGoal(2),
		bodyOrientation(0),
		bodyOrientation(1),
		bodyOrientation(2));
#ifdef IK_Debug
	cout << "fo_Transform: "<<endl<< "  "<<fOTransformationMatrix<<endl;
	cout << "co_Transform: "<<endl<< "  "<<cOTransformationMatrix<<endl;
#endif
	
	const Matrix4f fCTransform =(MMath::invertHomogenous(cOTransformationMatrix))
		*fOTransformationMatrix;
	
	const Matrix4f cFTransform =(MMath::invertHomogenous(fOTransformationMatrix))
		*cOTransformationMatrix;
#ifdef IK_Debug 
 	cout << "cf_Transform: "<<endl<< "  "<<fCTransform<<endl;
 	cout << "fc_Transform: "<<endl<< "  "<<cFTransform<<endl;
#endif
 	const float leg_sign = (chainID == LLEG_CHAIN )? 1.0f : -1.0f;
 /*значение w по умолчанию*/

 	//положение центра вращения таза относительно c
 	const Eigen::Vector4f hipOffset_c 
		(0.0f,leg_sign*HIP_OFFSET_Y,	-HIP_OFFSET_Z,1);
  	const Eigen::Vector4f ankleOffset_f 
		(0.0f,	0.0f,	FOOT_HEIGHT,1);
 

	
	const Eigen::Vector4f hipPosition_fprime =(cFTransform*hipOffset_c) - ankleOffset_f;
#ifdef IK_Debug

	cout<< "Hip position in fprime: "<< hipPosition_fprime<<endl;

#endif	
	const float legLength = hipPosition_fprime.norm();//длина

	const float legLengthSq = std::pow(legLength,2);

#ifdef IK_Debug
	/*ВАЖО! этот параметр добавлен для демонстраци работы кинематики 
	ккода происходит проверка с помощью клавиатуры  нельзя сменить срезу 
		две компоненты координаты, а изменяя только одну из них 
		мы гарантированно из этого положения попадаем в недопустимое и робот прест
		ает двигаться*/

	int DELTA=10; 
	if(legLength > THIGH_LENGTH+TIBIA_LENGTH+DELTA)
	
#endif

	if(legLength > THIGH_LENGTH+TIBIA_LENGTH)
	success=false;
#ifdef IK_Debug

		cout<< "LegLength: "<< legLength << ", sqrd = "<<legLengthSq<<endl;

#endif

	const float kneeCosine =
		(legLengthSq -TIBIA_LENGTH*TIBIA_LENGTH - THIGH_LENGTH*THIGH_LENGTH)/
		(2.0f*TIBIA_LENGTH*THIGH_LENGTH);
#ifdef IK_Debug

	cout<< "KneeCosine: "<<kneeCosine
		<< " unclipped cos"<< std::acos(kneeCosine)<<endl;

#endif

	const float KP = std::acos(std::min(std::max(kneeCosine,-1.0f),
		1.0f));
#ifdef IK_Debug

 	    cout<< "Calculated KP: "<<KP<<endl;
#endif
 	
 		const float AR = std::atan2(hipPosition_fprime(Kinematics::Y_AXIS),
 			hipPosition_fprime(Kinematics::Z_AXIS));
#ifdef IK_Debug

 		cout<< "Calculated AR: "<<AR<<endl;

#endif

 		const float pitch0 = std::asin(THIGH_LENGTH*std::sin(KP)/legLength);
 		const float AP =
 		std::asin(-hipPosition_fprime(Kinematics::X_AXIS)/legLength) - pitch0;
#ifdef IK_Debug

		cout<< "Calculated AP: "<<AP<<endl;
#endif
 		float tempHYP = givenHYPAngle;
//таз может быть как
		//входным параметром (для равновесия) так и 0
		//рyfqltv hyp если он не был найден
 	if(givenHYPAngle == HYP_NOT_SET){

 		
 		const Eigen::Matrix3f cf_Rot=cFTransform.block(0,0,3,3);

 		const Eigen::Matrix3f temp =
 			(Kinematics::rotation3D(Y_AXIS,AP+KP))*	(Kinematics::rotation3D(X_AXIS,AR));

 		const Eigen::Matrix3f cfh_Transform =temp*cf_Rot;

		if(chainID == LLEG_CHAIN){
			tempHYP =
				std::atan2(std::sqrt(2.0f)*cfh_Transform(Kinematics::Y_AXIS,
				Kinematics::X_AXIS),
				cfh_Transform(Kinematics::Y_AXIS,
				Kinematics::Y_AXIS) +
				cfh_Transform(Kinematics::Y_AXIS,
				Kinematics::Z_AXIS));



		}
		else{
			tempHYP =
				std::atan2(-std::sqrt(2.0f)*cfh_Transform(Kinematics::Y_AXIS,
				Kinematics::X_AXIS),
				cfh_Transform(Kinematics::Y_AXIS,
				Kinematics::Y_AXIS) -
				cfh_Transform(Kinematics::Y_AXIS,
				Kinematics::Z_AXIS));

 			}
	}


 	   const float HYP = tempHYP;

#ifdef IK_Debug
 	   cout<< "Calculated HYP: "<<HYP<<endl;
#endif

 	   const Eigen::Vector4f anklePosition_cprime = (fCTransform*
 		   ankleOffset_f) - hipOffset_c;

 	   const Eigen::Vector4f anklePosition_d = (chainID == LLEG_CHAIN ?
 												rotationHYPLeftInv(HYP) : 
 												 rotationHYPRightInv(HYP))
 													 * anklePosition_cprime;

 	const float HR = std::atan2(anklePosition_d(Kinematics::Y_AXIS),
 		-anklePosition_d(Kinematics::Z_AXIS));
 #ifdef IK_Debug
 	cout<< "Calculated HR: "<<HR<<endl;
#endif

 	const float pitch1 = std::asin(TIBIA_LENGTH*std::sin(KP)/legLength);
 	const float HP = std::asin(-anklePosition_d(Kinematics::X_AXIS)
 		/legLength) - pitch1;

#ifdef IK_Debug

 	 cout<< "Calculated HP: "<<HP<<endl;

#endif

	 res.angles[0] = HYP;
	 res.angles[1] = HR;
	 res.angles[2] = HP;
	 res.angles[3] = KP;
	 res.angles[4] = AP;
	 res.angles[5] = AR;


	 res.outcome = (success ? SUCCESS : STUCK);
	return res;
}

Eigen::Matrix4f Kinematics::rotationHYPLeftInv(const float HYP){
	float sinHYP, cosHYP;
	MMath::sincosf(HYP,&sinHYP,&cosHYP);
	const float sqrt2 = std::sqrt(2.0f);

	Eigen::Matrix4f r  = Eigen::Matrix4f::Identity();

	r(0,0) = cosHYP;
	r(0,1) = -sinHYP/sqrt2;
	r(0,2) = -sinHYP/sqrt2;

	r(1,0) = sinHYP/sqrt2;
	r(1,1) = 0.5f+cosHYP/2;
	r(1,2) = -0.5f+cosHYP/2;

	r(2,0) = sinHYP/sqrt2;
	r(2,1) = -0.5f+cosHYP/2;
	r(2,2) = 0.5f+cosHYP/2;

	return r;
}


Eigen::Matrix4f Kinematics::rotationHYPRightInv(const float HYP){
	float sinHYP, cosHYP;
	MMath::sincosf(HYP,&sinHYP,&cosHYP);
	const float sqrt2 = std::sqrt(2.0f);

	Eigen::Matrix4f r  = Eigen::Matrix4f::Identity();

	r(0,0) = cosHYP;
	r(0,1) = sinHYP/sqrt2;
	r(0,2) = -sinHYP/sqrt2;

	r(1,0) = -sinHYP/sqrt2;
	r(1,1) = 0.5f+cosHYP/2;
	r(1,2) = 0.5f-cosHYP/2;

	r(2,0) = sinHYP/sqrt2;
	r(2,1) = 0.5f-cosHYP/2;
	r(2,2) = 0.5f+cosHYP/2;

	return r;
}

const IKLegResult Kinematics::legIK( const ChainID chainID, const Eigen::Vector3f &footGoal, const Eigen::Vector3f &footOrientation, const Eigen::Vector3f &bodyGoal, const Eigen::Vector3f &bodyOrientation, const float HYPAngle /*= HYP_NOT_SET*/ )
{

	 IKLegResult result = analyticLegIK(chainID,footGoal,footOrientation,
		bodyGoal,bodyOrientation);

#ifdef DEBUG_WALK_IK
	 cout << "IK command with leg"<<chainID <<" :"<<endl
		 <<"    tried to put foot to "<<footGoal
		 << "      with orientation  "<<footOrientation<<endl
		 <<"    tried to put body to "<<bodyGoal
		 << "      with orientation  "<<bodyOrientation<<endl;
	 cout << "   result angles: {";
	 for(int i =0; i<6; i++){cout<<result.angles[i]<<",";}cout<<"}"<<endl;
#endif


	 if(result.outcome != Kinematics::SUCCESS){
		 cout << "IK ERROR with leg"<<chainID <<" :"
			 <<"    tried to put foot to "<<footGoal
			 << "      with orientation  "<<footOrientation<<endl
			 <<"    tried to put body to "<<bodyGoal
			 << "      with orientation  "<<bodyOrientation<<endl;
	 }

	  return result;

}
