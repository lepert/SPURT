#pragma once
#include "PlayerAction.hpp"
#include "Constants.hpp"
#include "Trigger.hpp"

enum ScenarioID
{
	S_Default ,
	S_Rush,
	S_FlankAttack,
	S_FullDefend,
	S_SimplePass,
	S_Race
};


class Scenario
{
public:
	Scenario(void);
	Scenario(int nID,char *nName);
	~Scenario(void);
	// название сценария
	char *pScenarioName;
	// id сценария
	int ID;
	// действия для всех игроков
	PlayerAction **PAction;
	void displayInfo(void);
	virtual  int NewStage(int PlayerID,int PlayerStage);
	virtual  int StagePlayer_1(int PlayerStage);
	virtual  int StagePlayer_2(int PlayerStage);
	virtual  int StagePlayer_3(int PlayerStage);
	virtual  int StagePlayer_0(int PlayerStage);


	virtual int ChooseActionForPlayer(int PlayerID,int PlayerStage);
	//Выбор действия для:
	virtual int ActionPlayer_1(int PlayerStage);	//Центр-форвард
	virtual int ActionPlayer_2(int PlayerStage);	//фланговый игрок (вингер)
	virtual int ActionPlayer_3(int PlayerStage);	//Защитник
	virtual int ActionPlayer_0(int PlayerStage);	//Вратарь
};






