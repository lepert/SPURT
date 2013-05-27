#ifndef PLAYER_HPP
#define PLAYER_HPP

//-----------------------------------------------------------------------------
//  File:         Player class (to be used in a Webots controllers)
//  Description:  Base class for FieldPlayer and GoalKeeper
//  Project:      Robotstadium, the online robot soccer competition
//  Author:       Yvan Bourquin - www.cyberbotics.com
//  Date:         May 4, 2008
//  Changes:      
//-----------------------------------------------------------------------------

#include <webots/Robot.hpp>
#include "PlayerState.hpp"
#include "WorldInfo.hpp"
#include "MessageQueue.h"
#include "Message.h"

namespace webots 
{
  class Accelerometer;
  class Gyro;
  class DistanceSensor;
  class LED;
  class TouchSensor;
  class Emitter;
  class Receiver;
  class GPS;
  class Motion;
  class Servo;
}

class NaoCam;

using namespace webots;
using namespace std;

class Player : public Robot 
{
public:
  Player(int playerID, int teamID);
  virtual ~Player();
  
  // find out which color to play
  bool isBlue() const;
  bool isRed() const;

  enum { NUM_TOUCH_SENSORS = 8 };

  // pure virtual: effective implementation in derived classes
  virtual void run() = 0;
  
  // overridden method
  virtual int step(int ms);

  // convert to string
  template <typename T>
  std::string toString (const T &t);

  template <typename T>
  T fromString (const std::string &str);

  // send
char* sendMessage(int /*idSender*/, int, Message /*msg*/, int /*idReceiver*/,  int /*broadcast*/);

char* readMessage (char*);
char* stringParam (char* );

protected:
  virtual void runStep();
  void playMotion(Motion *motion);
  void getUpIfNecessary();
  double getBallDirection() const;
  double getBallDistance() const;
  void headScan();
  void doAction();
  void CalcPlayerState();
  PlayerState *player_state;
  WorldInfo *world_info;
  

  // global control step (must be a multiple of WorldInfo.basicTimeStep)
  static const int SIMULATION_STEP;

  struct RoboCupGameControlData *gameControlData;

public:
  int idSender, idReceiver, broadcast; // (I'm add [if need be])
  MessageQueue* mq; // Queue Message
  int playerID, teamID;
  int nStage;	
protected:
  // devices
  NaoCam *camera;
  Servo *headYaw, *headPitch;
  Accelerometer *accelerometer;
  Gyro *gyro;
  DistanceSensor *topLeftUltrasound, *topRightUltrasound, *bottomLeftUltrasound, *bottomRightUltrasound;
  LED *chestLed, *rightEyeLed, *leftEyeLed, *rightEarLed, *leftEarLed, *rightFootLed, *leftFootLed;
  TouchSensor *fsr[NUM_TOUCH_SENSORS];  // force sensitive resistors
  Emitter *emitter, *superEmitter;
  Receiver *receiver;
  GPS *gps;  // for debugging only ! This device does not exist on the real robot.


  Motion *standUpFromFrontMotion;
  Motion *backwardsMotion, *forwardsMotion, *forwards50Motion, *turnRight40Motion, *turnLeft40Motion;
  Motion *turnRight60Motion, *turnLeft60Motion, *turnLeft180Motion, *sideStepRightMotion, *sideStepLeftMotion;
  Motion *shoot;


private:
  Camera *createCamera(const string&) const;
  void trackBall();
  void updateGameControl();
  void printTeamInfo(const struct TeamInfo *team);
  void printGameControlData(const struct RoboCupGameControlData *gcd);
  void readIncomingMessages();
  void sendInfoMessage();
  void sendMoveRobotMessage(double tx, double ty, double tz, double alpha);
  void sendMoveBallMessage(double tx, double ty, double tz);
  void sleepSteps(int steps);
  void loadMotions();

public:
	void SaveInfoMessage(int type,char *SMessage);
};

#endif
