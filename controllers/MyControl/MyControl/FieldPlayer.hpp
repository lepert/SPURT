#ifndef FIELD_PLAYER_HPP
#define FIELD_PLAYER_HPP

//-----------------------------------------------------------------------------
//  File:         FieldPlayer C++ class (to be used in a Webots controllers)
//  Description:  Field player (not a goalkeeper !)
//  Project:      Robotstadium, the online robot soccer competition
//  Author:       Yvan Bourquin - www.cyberbotics.com
//  Date:         May 8, 2009
//  Changes:      
//-----------------------------------------------------------------------------

#include "Player.hpp"

class FieldPlayer : public Player {
public:
  FieldPlayer(int playerID, int teamID);
  virtual ~FieldPlayer();

  // overridden function
  virtual void run();

protected:
  virtual void runStep();

private:
  // motion files


  // guessed goal direction (with respect to front direction of robot body)
  double goalDir;

  void turnBodyRel(double angle);
  void turnRight60();
  void turnLeft60();
  void turnRight40();
  void turnLeft40();
  void turnLeft180();

  void doAction();
  void CheckStage();
  void SendGPSPosition();
  void ReceiveGPSPosition();
};

#endif
