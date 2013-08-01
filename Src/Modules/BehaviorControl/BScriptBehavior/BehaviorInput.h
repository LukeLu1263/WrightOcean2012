#pragma once

#include "Representations/Modeling/FallDownState.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Tools/Settings.h"
#include "Input/KickInfoWrapper.h"
#include "Input/BallInfo.h"
#include "Input/GameInfoWrapper.h"
#include "Input/RobotInfoWrapper.h"
#include "Input/TeamInfoWrapper.h"
#include "Input/GoalWrapper.h"
#include "Input/KeyStatesWrapper.h"
#include "InputRepresentations.h"

struct BehaviorInput
{
  //TODO: refactor the whole input/output infrastructure!!
  void init(const InputRepresentations& inputReps)
  {
    kickInfo.init(inputReps);
  }

  void update(const InputRepresentations& inputReps)
  {
#ifdef TARGET_SIM
    preInitialEnabled = false;
#else
    preInitialEnabled = !Global::getSettings().recover;
#endif
    time = inputReps.theFrameInfo.time;
    ball.update(inputReps);
    keyStates.update(inputReps.theKeyStates);
    gameInfo.update(inputReps.theGameInfo);
    robotInfo.update(inputReps.theRobotInfo);
    teamInfo.update(inputReps.theOwnTeamInfo, inputReps.theOpponentTeamInfo);
    locator = inputReps.theRobotPose;
    motionInfo = inputReps.theMotionInfo;
    fallDownState = inputReps.theFallDownState;
    kickInfo.update(inputReps);
    goalWrapper.update(inputReps);
  }

  int               time;
  bool              preInitialEnabled;
  BallInfo          ball;
  KeyStatesWrapper  keyStates;
  GameInfoWrapper   gameInfo;
  RobotInfoWrapper  robotInfo;
  TeamInfoWrapper   teamInfo;
  RobotPose         locator;
  MotionInfo        motionInfo;
  FallDownState     fallDownState;
  KickInfoWrapper   kickInfo;
  GoalWrapper       goalWrapper;
};

