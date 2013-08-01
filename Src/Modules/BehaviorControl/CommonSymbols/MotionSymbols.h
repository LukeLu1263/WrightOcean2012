/**
* @file MotionSymbols.h
*
* Declaration of class MotionSymbols.
*
* @author Max Risler
*/

#pragma once

#include <cstdio>

#include "Modules/BehaviorControl/Symbols.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Tools/Xabsl/B-Human/BHXabslTools.h"

/**
* The Xabsl symbols that are defined in "motion_symbols.xabsl"
*
* @author Max Risler
*/
class MotionSymbols : public Symbols
{
public:
  MotionSymbols(MotionRequest& motionRequest, const MotionInfo& motionInfo, const RobotInfo& robotInfo) :
    motionRequest(motionRequest),
    motionInfo(motionInfo),
    robotInfo(robotInfo),
    lastSpecialAction(SpecialActionRequest::playDead)
  {}

private:
  MotionRequest& motionRequest; /**< A reference to the MotionRequest */
  const MotionInfo& motionInfo; /**< A reference to the MotionInfo */
  const RobotInfo& robotInfo; /**< A reference to the RobotInfo */

  SpecialActionRequest::SpecialActionID lastSpecialAction; /**< The last requested not-specialized special action. */

  /** registers the symbols at an engine */
  void registerSymbols(xabsl::Engine& engine);

  void update();

  void setWalkTargetX(float x);
  float getWalkTargetX();
  void setWalkTargetY(float y);
  float getWalkTargetY();
  void setMotionType(int type);
  int getMotionType();
  void setWalkSpeed(float speed);
  float getWalkSpeed();
  void setWalkTargetRot(float rot);
  float getWalkTargetRot();
  bool getWalkTargetReached();
  void setSpecialAction(int specialAction);
  int getSpecialAction();
  void setKickInWalk(int kickInWalk);
  int getKickInWalk();

  void setWalkSpeedX(float x);
  float getWalkSpeedX();
  void setWalkSpeedY(float y);
  float getWalkSpeedY();
  void setWalkSpeedRot(float rot);
  float getWalkSpeedRot();
  void setPercentageSpeedX(float percent);
  float getPercentageSpeedX();
  void setPercentageSpeedY(float percent);
  float getPercentageSpeedY();
  void setPercentageSpeedRot(float percent);
  float getPercentageSpeedRot();
};
