#ifndef COORD_FRAME_H
#define COORD_FRAME_H
#include <Eigen/Core>
#include <Eigen/Dense>
#include "Kinematics.h"

/**@breif  ������ 
���������� ��������������� ������
@detailed 
-������� �������� ����������
���������� ����� ������� 
��������� ������������ ������
*/

namespace Kinematics
{
	/**breif ������� ���������� ������� �������� 
	��� ���������� �������� �� ����������
	������� �������������� ���������
	@paramIn Axis ���, ������
	������� ����� ���� �������
	@return ������� 3�3 ��������
	*/
	const Eigen::Matrix3f rotation3D(const Axis axis,
	const float angle);

	/*
	
	@return Eigen::Matrix3f ������� ������������� 
	������������ �� ���� ��������� ������� 
	� ��������� ������� ������� (dx,dy,1)
 	*/
	const Eigen::Matrix3f translation3D(const float dx,
		const float dy);

}


#endif //COORD_FRAME_H