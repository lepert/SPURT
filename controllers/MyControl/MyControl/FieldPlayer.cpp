#include "FieldPlayer.hpp"
#include "NaoCam.hpp"
#include <webots/utils/Motion.hpp>
#include <webots/Servo.hpp>
#include <cmath>
#include <stdio.h>
#include <iostream>
using namespace webots;

FieldPlayer::FieldPlayer(int playerID, int teamID) : Player(playerID, teamID) {
  goalDir = 0.0; // interpolated goal direction (with respect to front direction of robot body)

  // move arms down
  getServo("LShoulderPitch")->setPosition(1.5);
  getServo("RShoulderPitch")->setPosition(1.5);
}

FieldPlayer::~FieldPlayer() {
  delete backwardsMotion;
  delete forwardsMotion;
  delete forwards50Motion;
  delete turnRight40Motion;
  delete turnLeft40Motion;
  delete turnRight60Motion;
  delete turnLeft60Motion;
  delete turnLeft180Motion;
  delete sideStepRightMotion;
  delete sideStepLeftMotion;
}

// normalize angle between -pi and +pi
static double normalizeAngle(double angle) {
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

// relative body turn
void FieldPlayer::turnBodyRel(double angle) {
  if (angle > 0.7)
    turnRight60();
  else if (angle < -0.7)
    turnLeft60();
  else if (angle > 0.3)
    turnRight40();
  else if (angle < -0.3)
    turnLeft40();
}

void FieldPlayer::runStep() {
  Player::runStep();
  double dir = camera->getGoalDirectionAngle();
  if (dir != NaoCam::UNKNOWN)
    goalDir = dir - headYaw->getPosition();
}

void FieldPlayer::turnRight60() {
  playMotion(turnRight60Motion); // 59.2 degrees
  goalDir = normalizeAngle(goalDir - 1.033);
}

void FieldPlayer::turnLeft60() {
  playMotion(turnLeft60Motion); // 59.2 degrees
  goalDir = normalizeAngle(goalDir + 1.033);
}

void FieldPlayer::turnRight40() {
  playMotion(turnRight40Motion); // 39.7 degrees
  goalDir = normalizeAngle(goalDir - 0.693);
}

void FieldPlayer::turnLeft40() {
  playMotion(turnLeft40Motion); // 39.7 degrees
  goalDir = normalizeAngle(goalDir + 0.693);
}

void FieldPlayer::turnLeft180() {
  playMotion(turnLeft180Motion); // 163.6 degrees
  goalDir = normalizeAngle(goalDir + 2.855);
}

void FieldPlayer::run() {
  step(SIMULATION_STEP);

  while (true) 
  {
    runStep();
    getUpIfNecessary();

    while (getBallDirection() == NaoCam::UNKNOWN)
	{
      std::cout << "searching the ball" << std::endl;
      getUpIfNecessary();
      if (getBallDirection() != NaoCam::UNKNOWN) break;
      headScan();
      if (getBallDirection() != NaoCam::UNKNOWN) break;
      playMotion(backwardsMotion);
      if (getBallDirection() != NaoCam::UNKNOWN) break;
      headScan();
      if (getBallDirection() != NaoCam::UNKNOWN) break;
      turnLeft180();
    }

    double ballDir = getBallDirection();
    double ballDist = getBallDistance();
	this->CalcPlayerState();

	//SendGPSPosition();
	//ReceiveGPSPosition();
	//CheckStage();
	doAction();


  }
}
void FieldPlayer::doAction()
{
	int current_action;
    double ballDist = getBallDistance();
	current_action=Kick/*world_info->pActiveScenario->ChooseActionForPlayer(this->playerID,this->nStage)*/;
	switch(current_action)
	{
	case Kick:
		//printf("ID #%i(st: #%i)ch kick [BD=%f]\n",this->playerID,this->nStage,ballDist);
		//fprintf(stdout,"ID #%i(st: #%i)ch kick [BD=%f]\n",this->playerID,this->nStage,ballDist);
		playMotion(shoot);
		break;
	case MoveForward:
		//printf("ID #%i(st: #%i) ch move f [BD=%f]\n",this->playerID,this->nStage,ballDist);
		playMotion(forwards50Motion);
		break;
	case Wait:
		//printf("ID #%i(st: #%i) ch wait [BD=%f]\n",this->playerID,this->nStage,ballDist);
		break;
	case DirectPass:
		playMotion(shoot);
		//printf("ID #%i(st: #%i) ch pass [BD=%f]\n",this->playerID,this->nStage,ballDist);
		break;
	case MoveSideLeft:
		playMotion(sideStepLeftMotion);
		//printf("ID #%i(st: #%i) ch move L [BD=%f]\n",this->playerID,this->nStage,ballDist);
		break;
	}
}

void FieldPlayer::CheckStage()
{
	double ballDist = getBallDistance();
	const double *Coord=this->gps->getValues();
	switch(playerID)
	{
	case 1:
		switch(nStage)
		{
		case 1:
			if(ballDist<0.2)
				nStage=2;
			break;
		case 2:
			if(ballDist>0.4)
				nStage=3;
			break;
		}
	break;
	case 2:
		switch(nStage)
		{
		case 1:
			if(ballDist<0.1)
				nStage=2;
			break;
		case 2:
			if(ballDist>0.4)
				nStage=3;
		}
		break;
	case 3:
		switch(nStage)
		{
		case 1:
			if(Coord[2]>=0)
			{
				nStage=2;
			}
			break;
		default:
			break;
		}
		break;
	}
}

void FieldPlayer::ReceiveGPSPosition()
{
	while (receiver->getQueueLength() > 0) 
	{
		const void *v= receiver->getData();
	
		char *t = new char[40];
		memcpy(t,v,40);
		/*printf("ID%i receive",playerID);*/
		//for(int i=0;i<40;i++)
		//	printf("%c",t[i]);
		//this->SaveInfoMessage(1,t);
	/*	printf("\n");*/
	}
}
void FieldPlayer::SendGPSPosition()
{
	const double *Coord=this->gps->getValues();
	char *t= new char[40];
	//sprintf(t,"ID%i\tX:%4f\tY:%4f\tZ:%4f\n",this->playerID,Coord[0],Coord[1],Coord[2]);
	//Message *nm = new Message(t,MessagePriority::medium,MessageType::mtGeoInform);
	//this->SaveInfoMessage(0,nm->getMessage());
	//emitter->send(t,40);
	
}

