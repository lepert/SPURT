#pragma once
#include "Scenario.hpp"

class SimplePass: public Scenario
{
	public:
	SimplePass(void);
	~SimplePass(void);
	int IsNewStage(int PlayerID,int PlayerStage);
	int StagePlayer_1(int PlayerStage);
	int StagePlayer_2(int PlayerStage);
	int StagePlayer_3(int PlayerStage);

	int ChooseActionForPlayer(int PlayerID,int PlayerStage);
	int ActionPlayer_1(int PlayerStage);
	int ActionPlayer_2(int PlayerStage);
	int ActionPlayer_3(int PlayerStage);
};