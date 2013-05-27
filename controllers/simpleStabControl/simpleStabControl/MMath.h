#ifndef MMATH_H
#define MMATH_H
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/triangular.hpp>
#include <boost/numeric/ublas/vector_proxy.hpp>

#define isnan(x)                                                        \
	(	sizeof (x) == sizeof(float )	?   __inline_isnanf((float)(x))	\
	:	sizeof (x) == sizeof(double)	?	__inline_isnand((double)(x)) \
	:	__inline_isnan ((long double)(x)))

namespace MMath{

	const float  cycloidx(const float theta);
	 const float  cycloidy(const float theta);

	 const Eigen::Matrix2f invert2by2(const Eigen::Matrix2f& m);
	//static const double M_PI;
	static const double PI = M_PI;
	static const double DEG_OVER_RAD = 180.0 / M_PI;//в градусы
	static const double RAD_OVER_DEG = M_PI / 180.0;//в радианы
	static const float M_PI_FLOAT = static_cast<float>(M_PI);

	static const float M_TO_CM  = 100.0f;
	static const float CM_TO_M  = 0.01f;
	static const float CM_TO_MM = 10.0f;
	static const float MM_TO_CM = 0.1f;

	
	static const float TO_DEG = 180.0f/M_PI_FLOAT;
	static const float TO_RAD = M_PI_FLOAT/180.0f;	
	static const float QUART_CIRC_RAD = M_PI_FLOAT / 2.0f;

	/**@breif функция вычисления матрицы 
	трансформации из фрейма  в фрейм
	@detailed матрица трансформации переводит
	координаты из одного базиса в другой
	@paramIn 
	-(xyz)-положение точки 
	-(wxwywz)-ориентация углы системы координат относительно базовой
	(см Introdution to robatics John J. p46)
	@return -матрица 4х4 перевода в базис
	*/
	const Eigen::Matrix4f get6fTransform(const float x,
		const float y,
		const float z,
		const float wx,
		const float wy,
		const float wz);

	const Eigen::Matrix4f invertHomogenous(const Eigen::Matrix4f &);

	/** @breif функция взятия синуса.
	@paramIn 
	-float _x параметр в радианах 
	@return _sinx _cosx*/

	inline static void sincosf(float _x, float * _sinx, float * _cosx) {
		*_sinx = std::sin(_x);
		*_cosx = std::cos(_x);
	}
	const float safe_asin(const float input);
	

	const float cliptoZero(const float value, const float minValue);
	const float clip(const float value, const float minValue,
		const float maxValue);
	const float clip(const float value, const float minMax);

	const float safe_atan2(const float y, const float x);


}
#endif