#include "Observer.h"


Observer::Observer(void):trackingError(0.0f)
{
	for (int i=0; i < 3; i++)
		stateVector(i) = 0.0f;
	

	for (int i=0; i < 3; i++)
		A(0, i) = A_values[i];
	for (int i=0; i < 3; i++)
		A(1, i) = A_values[3+i];
	for (int i=0; i < 3; i++)
		A(2, i) = A_values[6+i];

	for (int i=0; i < 3; i++)
		b[i] = b_values[i];

	for (int i=0; i < 3; i++)
		c[i]= c_values[i];

	for (int i=0; i < 3; i++)
		L[i] = L_values[i];


}
const float Observer::A_values[9] =
{  1.00000f,  0.01000f,  0.00000f,
0.37692f,  1.00000f, -0.37692f,
23.26017f, 3.78867f, -1.33361f };

const float Observer::b_values[3] =
{ 0.0f,
0.0f,
0.01f };

const float Observer::L_values[3] =
{ 0.042095f,
-0.338547f,
-0.274121f};

const float Observer::c_values[3] =
{ 0.0f, 0.0f, 1.0f };

const float Observer::Gi = -59.557f;

Observer::~Observer(void)
{
}
/**
Инициализация положения
скорости и углы,
а также ошибка траектории
 */
void Observer::initState(float x, float v, float p){
    stateVector(0) = x;
    stateVector(1) = v;
    stateVector(2) = p;
    trackingError = 0.0f;
}
/**
 * Расчет вектор-состояния zmp_ref
 *всегда берем х
 */
const float Observer::tick( const std::list<float> *zmp_ref, const float cur_zmp_ref, const float sensor_zmp )
{
	float preview_control = 0.0f;
	unsigned int counter = 0;

	for (std::list<float>::const_iterator i = zmp_ref->begin();
		counter < NUM_PREVIEW_FRAMES; ++counter, ++i) {
			preview_control += weights[counter]* (*i);
	}



    trackingError += ((c.transpose())*stateVector)(0) - cur_zmp_ref;


	const float control = -Gi * trackingError - preview_control;
	const float psensor = sensor_zmp;

	Eigen::Vector3f temp=(A* stateVector)
		- L*(psensor - (c.transpose()*stateVector)(0)) * 1.0f
		+ b*control;
	stateVector=(temp);
	
	return getPosition();



}

// веса из матлаба
const float Observer::weights[NUM_AVAIL_PREVIEW_FRAMES] =
{
	59.556590f, 95.02627f, 104.31770f, 103.40239f, 99.17544f, 94.03946f,
	88.80550f, 83.73909f, 78.91946f, 74.36278f, 70.06424f, 66.01249f,
	62.19447f, 58.59708f, 55.20770f, 52.01434f, 49.00569f, 46.17106f,
	43.50039f, 40.98421f, 38.61356f, 36.38004f, 34.27572f, 32.29311f,
	30.42519f, 28.66531f, 27.00722f, 25.44505f, 23.97324f, 22.58655f,
	21.28008f, 20.04918f, 18.88948f, 17.79686f, 16.76744f, 15.79756f,
	14.88379f, 14.02287f, 13.21174f, 12.44754f, 11.72754f, 11.04918f,
	10.41007f, 9.80792f, 9.24060f, 8.70610f, 8.20251f, 7.72806f,
	7.28104f, 6.85989f, 6.46309f, 6.08925f, 5.73703f, 5.40518f,
	5.09253f, 4.79796f, 4.52044f, 4.25896f, 4.01261f, 3.78051f,
	3.56183f, 3.35581f, 3.16170f, 2.97882f, 2.80651f, 2.64418f,
	2.49123f, 2.34713f, 2.21137f, 2.08345f, 1.96294f, 1.84940f,
	1.74242f, 1.64164f, 1.54668f, 1.45722f, 1.37293f, 1.29351f,
	1.21869f, 1.14820f, 1.08178f, 1.01921f, 0.96026f, 0.90471f,
	0.85238f, 0.80308f, 0.75663f, 0.71286f, 0.67163f, 0.63278f,
	0.59618f, 0.56169f, 0.52920f, 0.49859f, 0.46975f, 0.44258f,
	0.41698f, 0.39286f, 0.37014f, 0.34873f, 0.32855f, 0.30955f,
	0.29164f, 0.27478f, 0.25888f, 0.24391f, 0.22980f, 0.21651f,
	0.20398f, 0.19218f, 0.18107f, 0.17059f, 0.16073f, 0.15143f,
	0.14267f, 0.13442f, 0.12664f, 0.11932f, 0.11242f, 0.10591f
};