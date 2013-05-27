#include <cstdio>
#include "WorldInfo.hpp"

WorldInfo::WorldInfo(void)
: score(0)
, pBallPosition(NULL)
{
	pBallPosition = new MyPoint();
	pAllyState = new PlayerState*[Players_in_team];
	pEnemyState = new PlayerState*[Players_in_team];
}
WorldInfo::~WorldInfo(void)
{
	delete pAllyState;
	delete pEnemyState;
}


float WorldInfo::mathDangerLevel()
{
	float MaxDangerEnemy = 0.0f;
	float MaxDangerAlly = 0.0f; // пока не используется
	float CurrentDanger = 0.0f;
	for(int i=0; i< Players_in_team; i++)
	{
		 CurrentDanger = pEnemyState[i]->PlayerPosition->distance2DTo(pBallPosition);
		 if(CurrentDanger > MaxDangerEnemy)
			 MaxDangerEnemy = CurrentDanger;
	}

	/*for(int i=0; i < Players_in_team; i++)
	{
		 CurrentDanger = pEnemyState[i]->PlayerPosition->distance2DTo(pBallPosition);
		 if(CurrentDanger>MaxDangerEnemy)
			 MaxDangerEnemy=CurrentDanger;
	}*/
	return 0.0f;
}

int WorldInfo::chooseScenario(void)
{

	return chooseScenarioByScore();
}

int WorldInfo::chooseScenarioByScore(void)
{
	Scenario *AScenario;
	if(this->score>=3)
	{
		AScenario = new FullDefence();
		this->pActiveScenario=AScenario;
		AScenario->displayInfo();
		return AScenario->ID;
	}
	if(this->score<=-2)
	{
		AScenario = new Rush();
		this->pActiveScenario=AScenario;
		AScenario->displayInfo();
		return AScenario->ID;
	}
	AScenario = new FlankAttack();
	this->pActiveScenario=AScenario;
	AScenario->displayInfo();
	return AScenario->ID;
}



int WorldInfo::chooseScenarioByAllyPosition(void)
{
	Scenario *AScenario;
/* 
число полевых игроков, за чужой половине
	int NPlayersOnEnemyHalf=0;
	// число полевых игроков на чужом фланге
	int NPlayersOnEnemyFlanks=0;
*/      
        // число полевых игроков, за чужой половине
	int nPlayersOnEnemyHalf = 0;
	// число полевых игроков на чужом фланге
	int nPlayersOnEnemyFlanks = 0;
	// число полевых игроков у ворот(во вратарской)
	int nPlayersInEnemyGoalKeaperArea = 0;
	// число полевых игроков в чужой штрафной зоне
	int nPlayersInEnemyPenaltyArea = 0;

	// Вратаря не считаем, его index=0
	for(int i=1;i<Players_in_team;i++)
	{
		switch(pAllyState[i]->getPositionZone())
		{
			case TOP_LEFT_FLANK:
				break;

			case TOP_RIGHT_FLANK:
				nPlayersOnEnemyHalf++;
				nPlayersOnEnemyFlanks++;
				break;

			case LEFT_GOALKEAPER_AREA:
				break;

			case RIGHT_GOALKEAPER_AREA:
				nPlayersOnEnemyHalf++;
				nPlayersInEnemyGoalKeaperArea++;
				break;

			case DOWN_LEFT_FLANK:
				break;

			case DOWN_RIGHT_FLANK:
				nPlayersOnEnemyHalf++;
				nPlayersOnEnemyFlanks++;
				break;

			case LEFT_PENALTY_AREA:
				break;

			case RIGHT_PENALTY_AREA:
				nPlayersOnEnemyHalf++;
				nPlayersInEnemyPenaltyArea++;
				break;

			case LEFT_CENTER:
				break;

			case RIGHT_CENTER:
				nPlayersOnEnemyHalf++;
				break;

			
			case UNKNOWN:
			case OUT:
			default:
				break;
		}
	}
	if(nPlayersOnEnemyHalf>3)
	{
		AScenario = new Rush();
		this->pActiveScenario=AScenario;
		AScenario->displayInfo();
		return AScenario->ID;
	}

	if(nPlayersOnEnemyHalf== 0)
	{
		AScenario = new FullDefence();
		this->pActiveScenario=AScenario;
		AScenario->displayInfo();
		return AScenario->ID;
	}

	if(nPlayersOnEnemyHalf>=2 || nPlayersOnEnemyFlanks>=1)
	{
		AScenario = new FlankAttack();
		this->pActiveScenario=AScenario;
		AScenario->displayInfo();
		return AScenario->ID;
	}
	AScenario= new Scenario();
	return AScenario->ID;
}
