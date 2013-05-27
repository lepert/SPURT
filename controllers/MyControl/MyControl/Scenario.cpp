#include <cstdio>
#include "Scenario.hpp"

Scenario::Scenario(void)
{
	PAction=new PlayerAction*[Players_in_team];
	ID=S_Default;
	pScenarioName = "Default Empty";
}

Scenario::~Scenario(void)
{
	delete PAction;
}


Scenario::Scenario(int nID,char *nName)
{
	PAction=new PlayerAction*[Players_in_team];
	this->pScenarioName=nName;
	this->ID=nID;
	
}


int Scenario::ChooseActionForPlayer(int,int)
{
	return Actions::Wait;
}

void Scenario::displayInfo(void)
{

	char ch = pScenarioName[0];
	int count = 0;
	while(ch != '\0' || ch != NULL)
	{
	
		ch = pScenarioName[count];
	}
	
}



int Scenario::ActionPlayer_1(int PlayerStage)
	{
		return Actions::Wait;
	}
int Scenario::ActionPlayer_2(int PlayerStage)
	{
		return Actions::Wait;
	}
int Scenario::ActionPlayer_3(int PlayerStage)
	{
		return Actions::Wait;
	}
int Scenario::ActionPlayer_0(int PlayerStage)
	{
		return Actions::Wait;
	}


int Scenario::NewStage(int PlayerID, int PlayerStage)
{
	return PlayerStage;
}

int Scenario::StagePlayer_1(int PlayerStage)
{
	return PlayerStage;
}

int Scenario::StagePlayer_2(int PlayerStage)
{
	return PlayerStage;
}

int Scenario::StagePlayer_3(int PlayerStage)
{
	return PlayerStage;
}

int Scenario::StagePlayer_0(int PlayerStage)
{
	return false;
}





