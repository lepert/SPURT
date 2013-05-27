#ifndef SAMPLEROBOT_H
#define SAMPLEROBOT_H

#include "ForwardKinematics.h"
#include "MMath.h"
#include <Eigen/Core>
#include <Eigen/Dense>
#include "InverseKinematics.h"
#include "StepGenerator.h"
#include "WBSensorsTranslator.h"
#include "WalkingLeg.h"
#include <vector>
#include "GeneralSensors.h"
#include <webots/robot.h>

#define IK_GoalControl
class StepGenerator;

class WBSensorsTranslator;

class SampleRobot 
{
public:
	SampleRobot();
	~SampleRobot();

/**@breif ������� ���� ����������
@detailed ����������� ���� ���������
�� ���������� ���� ���������*/

	void run();
	/**@breif ������� ��� ������������ � ������� ����������
	@detailed ����� ��������� ��� �������� �������� ���������� 
	����� ��������� ������� � ����������
	� ������� ������� ������� �������
	������� ����� ���� ������ 
	��� + �������� ��
	10 �� �� ����������� ��z (footgoal)
	���������� ���������� �� �����
	������� ����� ���� ������ NUMPAD 
	(�������� ����� � ���������� NuMLOCK)
	��� - �������� ��
	10 �� �� ����������� ��z (footgoal)
	*/
	//Eigen::Vector3f runKeyCommand(const Eigen::Vector3f,const int)const;

private:
	int counter;
	std::tr1::shared_ptr<GeneralSensors> generalSensors;
	 std::tr1::shared_ptr<StepGenerator> stepGenerator;
	 std::tr1::shared_ptr<WBSensorsTranslator> wbsensorsTranslator;
	  std::vector<WbDeviceTag> jointDevices;
	      std::vector<float> motionValues;

};

#endif //SAMPLEROBOT_H