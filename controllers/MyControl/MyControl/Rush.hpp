#pragma once
#include "Scenario.hpp"

class Rush: public Scenario
{
public:
	Rush(void);
	~Rush(void);
	int ChooseActionForPlayer(int PlayerID,int PlayerStage);
	int ActionPlayer_1(int PlayerStage);
	int ActionPlayer_2(int PlayerStage);
	int ActionPlayer_3(int PlayerStage);
	int ActionPlayer_0(int PlayerStage);
};