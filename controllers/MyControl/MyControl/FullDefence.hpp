#pragma once
#include "Scenario.hpp"

class FullDefence: public Scenario
{
	public:
	FullDefence(void);
	~FullDefence(void);
	int ChooseActionForPlayer(int PlayerID,int PlayerStage);
};