#pragma once
#include "Scenario.hpp"



class FlankAttack: public Scenario
{
public:
	FlankAttack(void);
	~FlankAttack(void);
	int ChooseActionForPlayer(int PlayerID,int PlayerStage);
	int ActionPlayer_1(int PlayerStage);
	int ActionPlayer_2(int PlayerStage);
	int ActionPlayer_3(int PlayerStage);
	int ActionPlayer_0(int PlayerStage);
};