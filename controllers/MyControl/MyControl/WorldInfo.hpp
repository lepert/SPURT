#pragma once
#include "PlayerState.hpp"
#include "Constants.hpp"
//#include "Scenario.hpp"
#include "SimplePass.hpp"
#include "FlankAttack.hpp"
#include "Rush.hpp"
#include "Race.hpp"
#include "FullDefence.hpp"

class WorldInfo
{
public:
	WorldInfo(void);
	~WorldInfo(void);


	
	// Разница забитых-пропущенных мячей в игре
	int score;

	// Координаты мяча
	MyPoint *pBallPosition;
	// Состояния союзников
	PlayerState **pAllyState;
	// Состояния соперников
	PlayerState **pEnemyState;

	// степень опасности игрока
	float mathDangerLevel();
	// Текущий сценарий
	Scenario *pActiveScenario;
	int chooseScenario(void);
	int chooseScenarioByScore(void);
	// Выбор сценария по позиции своих игроков
	int chooseScenarioByAllyPosition(void);
	
};
