#ifndef FORWARD_KINEMATICS_H
#define FORWARD_KINEMATICS_H

#include "Kinematics.h"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <vector>
using namespace Eigen;
using namespace MMath;

/**@breif  расчет прямой кинематики
@detailed примая кинематика это набор 
методов определяющих точку 
конечного сочленения относительно
центра тяжести робота пупка

пример тестирования :
повернем правую ногу так, чтобы угол
м.д ногами составил 90градусов
sensors->getReadyServoByName(JOINT_STRINGS[14])->setPosition(-90*RAD_OVER_DEG);
запишем показания углов для ноги (проверка Р регулятора)
sensors->getArrayServoSensors(anglesServo,RLEG_SERVOS);
вывод углов
std::cout<<"angles";
for (unsigned int i=0;i<anglesServo.size();i++){
std::cout<<" "<<anglesServo[i]*MMath::DEG_OVER_RAD;}
std::cout<<std::endl;
angles 0.0001656 9.73e-005 -90 -8.295e-005 -5.528e-005 5.23e-005
std::cout<<"coord"<<Kinematics::forwardKinematics(Kinematics::RLEG_CHAIN,anglesServo)<<"/coord"<<std::endl;
[simpleStabControl] coord246
[simpleStabControl] -50
[simpleStabControl] -85/coord
расчт верен, вне зависимости от того,
что , казалось бы z должен быть равен 0
все дело в том , что существует параметр смещения 
static const float HIP_OFFSET_Z = 85.0f;
благодаря ему координаты расчитываются 
не от начала (RHipYawPitch) а от центра масс

*/
namespace Kinematics
{

	const Vector3d forwardKinematics(const ChainID id,	const std::vector<float> & ang);

}
#endif //FORWARD_KINEMATICS_H