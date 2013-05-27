#pragma once
#include "Scenario.hpp"


class Race : public Scenario
{
public:
	Race();
	~Race();
	int ChooseActionForPlayer(int PlayerID,int PlayerStage);
};