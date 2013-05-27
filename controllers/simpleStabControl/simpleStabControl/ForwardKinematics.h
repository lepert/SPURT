#ifndef FORWARD_KINEMATICS_H
#define FORWARD_KINEMATICS_H

#include "Kinematics.h"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <vector>
using namespace Eigen;
using namespace MMath;

/**@breif  ������ ������ ����������
@detailed ������ ���������� ��� ����� 
������� ������������ ����� 
��������� ���������� ������������
������ ������� ������ �����

������ ������������ :
�������� ������ ���� ���, ����� ����
�.� ������ �������� 90��������
sensors->getReadyServoByName(JOINT_STRINGS[14])->setPosition(-90*RAD_OVER_DEG);
������� ��������� ����� ��� ���� (�������� � ����������)
sensors->getArrayServoSensors(anglesServo,RLEG_SERVOS);
����� �����
std::cout<<"angles";
for (unsigned int i=0;i<anglesServo.size();i++){
std::cout<<" "<<anglesServo[i]*MMath::DEG_OVER_RAD;}
std::cout<<std::endl;
angles 0.0001656 9.73e-005 -90 -8.295e-005 -5.528e-005 5.23e-005
std::cout<<"coord"<<Kinematics::forwardKinematics(Kinematics::RLEG_CHAIN,anglesServo)<<"/coord"<<std::endl;
[simpleStabControl] coord246
[simpleStabControl] -50
[simpleStabControl] -85/coord
����� �����, ��� ����������� �� ����,
��� , �������� �� z ������ ���� ����� 0
��� ���� � ��� , ��� ���������� �������� �������� 
static const float HIP_OFFSET_Z = 85.0f;
��������� ��� ���������� ������������� 
�� �� ������ (RHipYawPitch) � �� ������ ����

*/
namespace Kinematics
{

	const Vector3d forwardKinematics(const ChainID id,	const std::vector<float> & ang);

}
#endif //FORWARD_KINEMATICS_H