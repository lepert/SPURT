#include "FlankAttack.hpp"

FlankAttack::FlankAttack():Scenario(S_FlankAttack,"Flank Attack")
{
}

FlankAttack::~FlankAttack()
{
}
int FlankAttack::ChooseActionForPlayer(int PlayerID,int PlayerStage)
{
	return 0;
}

	int FlankAttack::ActionPlayer_1(int PlayerStage)
	{
		return Actions::Wait;
	}
	int FlankAttack::ActionPlayer_2(int PlayerStage)
	{
		return Actions::Wait;
	}
	int FlankAttack::ActionPlayer_3(int PlayerStage)
	{
		return Actions::Wait;
	}
	int FlankAttack::ActionPlayer_0(int PlayerStage)
	{
		return Actions::Wait;
	}