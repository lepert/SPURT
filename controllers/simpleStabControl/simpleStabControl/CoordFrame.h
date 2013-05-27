#ifndef COORD_FRAME_H
#define COORD_FRAME_H
#include <Eigen/Core>
#include <Eigen/Dense>
#include "Kinematics.h"

/**@breif  модуль 
нахождения вспомогательный матриц
@detailed 
-матрица вращения определяет
ориентацию одной системы 
координат относительно другой
*/

namespace Kinematics
{
	/**breif функция нахождения матрицы вращения 
	или компоненты вращения из гомогенной
	матрицы преобразования координат
	@paramIn Axis ось, вокруг
	которой задан угол вращеия
	@return матрица 3х3 вращения
	*/
	const Eigen::Matrix3f rotation3D(const Axis axis,
	const float angle);

	/*
	
	@return Eigen::Matrix3f матрица трансформации 
	представляет из себя единичную матрицу 
	в последнем столбце которой (dx,dy,1)
 	*/
	const Eigen::Matrix3f translation3D(const float dx,
		const float dy);

}


#endif //COORD_FRAME_H