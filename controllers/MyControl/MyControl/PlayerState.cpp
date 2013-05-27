
#include "PlayerState.hpp"

PlayerState::PlayerState(void)
: PlayerPosition(0)
, BallDistance(0)
, AngleView(0)
, AngleBody(0)
{
}

PlayerState::~PlayerState(void)
{
}



PlayerState::PlayerState(MyPoint * InitialPosition)
{
	PlayerState();
	this->PlayerPosition=InitialPosition;
}
PlayerState::PlayerState(int  InitialState)
{
	PlayerState();
	//проверка на возможность статуса - ПРОПУЩЕНА
	
	this->CurState=InitialState;
}
// Определяет, находится ли игрок на фланге
int PlayerState::isOnFlank(void)
{
	float px,py;
	px=this->PlayerPosition->cX;
	py=this->PlayerPosition->cY;

	if(px<3 && px>= 0 && py >=1 &&py < 2)
		return TOP_LEFT_FLANK;
	if(px>3 && px>= 0 && py >=1 &&py < 2)
		return TOP_LEFT_FLANK;
	return UNKNOWN;
}


bool PlayerState::isPlayerOut(void)
{
	float px,py;
	px=this->PlayerPosition->cX;
	py=this->PlayerPosition->cY;
	if(px <-3 || px >3 ||py <-2  || py > 2)
		return true;
	else
		return false;

}


// 
int PlayerState::getPositionZone(void)
{
	if(isPlayerOut()==true)
		return OUT;
	if(isOnFlank()!=UNKNOWN)
		return isOnFlank();

	return UNKNOWN;
}


char *PlayerState::GenerateDescription()
{
	char *TextState= new char[200];
	sprintf(TextState,"ID:%i Coord[%.4f,%.4f,%.4f] ball_dist=%.4f",this->PlayerID,PlayerPosition->cX,PlayerPosition->cY,PlayerPosition->cZ
		,this->BallDistance);

	return TextState;
}


void PlayerState::WriteStateToFile()
{
	//char* filename=new char[9];
	//sprintf(filename,"S%2d.txt",PlayerID);
	//FILE* statefile=fopen(filename,"a+");
	//fprintf(statefile,"%s\n",GenerateDescription());
	//fclose(statefile);
}