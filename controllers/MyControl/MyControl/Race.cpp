#include "Race.hpp"


//Класс Race
Race::Race():Scenario(S_Race,"Race")
{

}

Race::~Race()
{
}

int Race::ChooseActionForPlayer(int PlayerID, int PlayerStage)
{
	return Actions::Wait;
}