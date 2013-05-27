#pragma once


enum Actions 
{
	Wait,
	Kick,
	MoveForward,
	MoveSideLeft,
	MoveSideRight,
	Rotate,
	DirectPass,
	
};
class PlayerAction
{
public:
	PlayerAction(void);
	~PlayerAction(void);
	bool MoveTo(float X, float Y);
	bool Kick(float X,float Y, float force);
	int ID;
};
