#include <iostream>
#include <stdio.h>

#include "SampleRobot.h"

#define TIME_STEP 40


int main(int argc, const char *argv[]) {
	
	if (argc < 3) {
		cout << "Error: could not find teamID and playerID in controllerArgs" << endl;
		return 0;
	}

wb_robot_init();

system("title");
int playerID = atoi(argv[1]);
int teamID = atoi(argv[2]);



	std::auto_ptr<SampleRobot> naoSample(new SampleRobot());
	/*int timeStep =static_cast<int>(naoSample->getBasicTimeStep());*/
	
	int time_step = static_cast<int>(wb_robot_get_basic_time_step());

	
	while(1)
	{  
		wb_robot_step(time_step);

/*@breif продолжительность шага симуляции Webots
@detailed обновляет данные симулятора каждые timeStep миллисекунд
если значние очень высоко то расчет будет грубый
@paramin значение в описании word basicTimeStep 40
*/

	
	naoSample->run();
	
	


	}

	
	return 0;
}