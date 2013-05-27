#include "Rush.hpp"

//Класс Rush
Rush::Rush():Scenario(S_Rush,"Rush")
{
	
}

Rush::~Rush()
{

}


int Rush::ChooseActionForPlayer(int PlayerID,int PlayerStage)
{
	return 0;
}

int Rush::ActionPlayer_1(int PlayerStage)
{
	return Actions::Wait;
}
int Rush::ActionPlayer_2(int PlayerStage)
{
	return Actions::Wait;
}
int Rush::ActionPlayer_3(int PlayerStage)
{
	return Actions::Wait;
}
int Rush::ActionPlayer_0(int PlayerStage)
{
	return Actions::Wait;
}