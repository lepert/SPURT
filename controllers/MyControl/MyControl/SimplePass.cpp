#include "SimplePass.hpp"

//Класс SimplePass
SimplePass::SimplePass():Scenario(S_SimplePass,"Simple Pass")
{

}

SimplePass::~SimplePass()
{

}

int SimplePass::StagePlayer_1(int PlayerStage)
{
	switch(PlayerStage)
		{
		case 1:
			//if(ballDist<0.2)
				return 2;
			break;
		case 2:
			//if(ballDist>0.4)
				return 3;
			break;
		}
	return PlayerStage;
}
int SimplePass::StagePlayer_2(int PlayerStage)
{
	return PlayerStage;
}

int SimplePass::StagePlayer_3(int PlayerStage)
{
	return PlayerStage;
}
int SimplePass::IsNewStage(int PlayerID,int PlayerStage)
{
	if(PlayerID==1)
	{
		return StagePlayer_1(PlayerStage);
	}
	if(PlayerID==2)
	{
		return StagePlayer_2(PlayerStage);
	}
	if(PlayerID==3)
	{
		return StagePlayer_3(PlayerStage);
	}
	return false;
}
int SimplePass::ActionPlayer_1(int PlayerStage)
{
	switch(PlayerStage)
		{
		case 1:
			return Actions::MoveForward;
			break;
		case 2:
			return Actions::DirectPass;
			break;
		case 3:
			return Actions::Wait;
			break;
		default:
			return Actions::Wait;
		}
}
int SimplePass::ActionPlayer_2(int PlayerStage)
{
		switch(PlayerStage)
		{
		case 1:
			return Actions::MoveForward;
			break;
		case 2:
			return Actions::Wait;
			break;
		case 3:
			return Actions::Kick;
			break;
		default:
			return Actions::Wait;
		}
}
int SimplePass::ActionPlayer_3(int PlayerStage)
{
		switch(PlayerStage)
		{
			case 1:
				return Actions::MoveSideLeft;
				break;
			default:
				return Actions::Wait;
				break;
		}
}
int SimplePass::ChooseActionForPlayer(int PlayerID,int PlayerStage)
{
	if(PlayerID==1)
	{
		return ActionPlayer_1(PlayerStage);	
	}
	if(PlayerID==2)
	{
		return ActionPlayer_2(PlayerStage);
	}
	if(PlayerID==3)
	{
		return ActionPlayer_3(PlayerStage);
	}
	return Actions::Wait;
}
