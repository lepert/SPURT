/**
@breif ������ ���������
O-������� ���������� ���������
F-�� ����� ��� �����
�-�����
*/

#ifndef INVERSE_KINEMATICS_H 
#define INVERSE_KINEMATICS_H
#include <Eigen/Core>
#include <Eigen/Dense>
#include <vector>
#include "Kinematics.h"
#include "CoordFrame.h"

using namespace Eigen;
using namespace Kinematics;
using namespace std;

/**@breif ������ �������� ����������
@detailed �������� ���������� 
���������� �������� ����� 
���������� ��� ����������
������������ �����
*/
namespace Kinematics
{
	/**@breif ������� ���������� ����� ����������
	@paramIn 
	- chainID, ���������� 
	- &footGoal, ������ 
	���������� ����� - ����
	- &footOrientation, 
	������ ����������� ����������� 
	���� (000)-����������� �����
	��������� ��������� ��� ����������� ����������
	- &bodyGoal, 
	- &bodyOrientation, 
	- givenHYPAngle
	@return IKResult - ��������� � ������ � bool ��������� 
	���� ���������� ���� ������ �� ���� ���
	, ��� ����� ������ �� ����� ������� �� �����
	�� ������ ����� ����� ������� ���� �������
	� ���� ������ ���� �������� 
	���������� ������� � ����������� ������
	!!!�����! ����� �� ��������
	���� ����� �� try cath � ����� �� �������?
	*/
	/*
	#ifdef IK_GoalControl

	static Eigen::Vector3f footGoal(0,50,-331);
	const Eigen::Vector3f footOrientation(0,0,0);
	const Eigen::Vector3f bodyGoal(0,0,0);
	const Eigen::Vector3f bodyOrientation(0,0,0);

	int newKey=this->keyboardGetKey();
	Eigen::Vector3f precalculatedAchivableGoal=runKeyCommand(footGoal,newKey);



	std::vector<float> anglesServo;
	sensors->getArrayServoSensors(anglesServo,LLEG_SERVOS);
	std::cout<<"coord"<<Kinematics::forwardKinematics(Kinematics::LLEG_CHAIN,anglesServo)<<"/coord"<<std::endl;
	const Kinematics::IKResult res=analyticLegIK(Kinematics::LLEG_CHAIN,precalculatedAchivableGoal,footOrientation,bodyGoal,bodyOrientation,anglesServo[0]);

	if (res.successfulIK)
	{
	sensors->getReadyServoByName(JOINT_STRINGS[6])->setPosition(res.angles[3]);
	sensors->getReadyServoByName(JOINT_STRINGS[7])->setPosition(res.angles[4]);
	sensors->getReadyServoByName(JOINT_STRINGS[8])->setPosition(res.angles[5]);
	sensors->getReadyServoByName(JOINT_STRINGS[9])->setPosition(res.angles[0]);
	sensors->getReadyServoByName(JOINT_STRINGS[10])->setPosition(res.angles[2]);
	sensors->getReadyServoByName(JOINT_STRINGS[11])->setPosition(res.angles[1]);
	footGoal=precalculatedAchivableGoal;	
	}

	#endif*/
   static const float HYP_NOT_SET = -1000.0f;

	enum IKOutcome {
		STUCK = 0,
		SUCCESS = 1
	};

	struct IKLegResult {
		IKOutcome outcome;
		float angles[6];
	};

	const IKLegResult legIK(const ChainID chainID,
		const Eigen::Vector3f &footGoal,
		const Eigen::Vector3f &footOrientation,
		const Eigen::Vector3f &bodyGoal,
		const Eigen::Vector3f &bodyOrientation,
		const float HYPAngle = HYP_NOT_SET);

	const Kinematics::IKLegResult analyticLegIK(const ChainID chainID,
		const Vector3f &footGoal,
		const Vector3f &footOrientation,
		const Vector3f &bodyGoal,
		const Vector3f &bodyOrientation,
		const float givenHYPAngle = HYP_NOT_SET);
	/**��������������� ������� 
	���������� ������ �������� ��� �����*/
	Eigen::Matrix4f rotationHYPLeftInv(const float );
	Eigen::Matrix4f rotationHYPRightInv(const float );

}
#endif //INVERSE_KINEMATICS_H