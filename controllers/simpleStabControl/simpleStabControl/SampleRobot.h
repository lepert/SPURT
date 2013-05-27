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

/**@breif главный цикл выполнения
@detailed бесконечный цикл действует
на протяжении всей программы*/

	void run();
	/**@breif функция для тестирования с помощью клавиатуры
	@detailed чтобы посмотеть как работает обратная кинематика 
	можно управлять роботом с клавиатуры
	в функции описаны команды стрелок
	стрелки влево вниз вправо 
	это + значений на
	10 мм по координатам хуz (footgoal)
	координаты отсчитывай от пупка
	стрелки влево вниз вправо NUMPAD 
	(цифровая клава с выключеным NuMLOCK)
	это - значений на
	10 мм по координатам хуz (footgoal)
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