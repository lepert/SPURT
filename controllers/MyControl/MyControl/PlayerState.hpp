#pragma once
#include "MyPoint.hpp"


// мяч 
enum BallState
{
	HaveBall=1,
	NearBall=2,
	SeeBall=3,
	FarAwayBall=4,
	DontSeeBall=5,
};
// список состояний
enum  States
{
	Run=1,
	Stay=2,
	Fall=3,
	standUp=4,

};
// насколько опасен соперник / насколько полезен союзник
enum Danger
{
	NoDanger=0,
	LowDanger=1,
	MiddleDanger=2,
	Dange=3,
	HighDanger=4,
	ExtremelyDanger=5,
};
//  Пoложение игрока
enum FieldPosition
{
	UNKNOWN = -1,// положение неизвестно, или не важно
	OUT = 0,	 // вне поля
	TOP_LEFT_FLANK=1, // Верхний левый (КРАСНЫЙ) фланг
	TOP_RIGHT_FLANK=2, // Верхний правый (СИНИЙ) фланг
	LEFT_GOALKEAPER_AREA=3,
	RIGHT_GOALKEAPER_AREA=4,
	DOWN_LEFT_FLANK=5, // Нижний левый (КРАСНЫЙ) фланг
	DOWN_RIGHT_FLANK=6, // Нижний правый (СИНИЙ) фланг
	LEFT_PENALTY_AREA=7,
	RIGHT_PENALTY_AREA=8,
	LEFT_CENTER=9,
	RIGHT_CENTER=10,


};


class PlayerState
{
public:
	PlayerState(void);
	PlayerState(MyPoint * InitialPosition);
	PlayerState(int  InitialState);
	~PlayerState(void);
	// текущее состояние
	int CurState;
	// координаты игрока
	MyPoint *PlayerPosition;

	// расстояние до мяча
	float BallDistance;

	int plFieldPos;
	// Определяет, находится ли игрок на фланге
	int isOnFlank(void);
	// Зона на поле, где располагается игрок
	int getPositionZone(void);
	// Вышел ли игрок за пределы поля
	bool isPlayerOut(void);
	// угол направления взгляда игрока
	int AngleView;
	// угол поворота тела игрока
	float AngleBody;
	//Идентификатор игрока
	int PlayerID;

	// Номер этапа сценария для игрока
	int nStage;
	// ID команды
	int TeamID;

private: 
	char * GenerateDescription();
public:
	void WriteStateToFile();
};
