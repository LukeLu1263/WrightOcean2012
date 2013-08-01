#pragma once

#include "Representations/Infrastructure/GameInfo.h"

struct GameInfoWrapper
{
  enum State
  {
    initial = STATE_INITIAL,
    ready = STATE_READY,
    set = STATE_SET,
    playing = STATE_PLAYING,
    finished = STATE_FINISHED,
  };

  enum SecondaryState
  {
    normal = STATE2_NORMAL,
    penaltyshoot = STATE2_PENALTYSHOOT,
  };

  enum Team
  {
    blue = TEAM_BLUE,
    red = TEAM_RED,
  };

  State state;
  SecondaryState secondaryState;
  bool firstHalf;
  Team kickOffTeam,
       dropInTeam;
  int dropInTime,
      secsRemaining,
      timeSinceLastPackageReceived;

  void update(const GameInfo& gameInfo)
  {
    state = (State)gameInfo.state;
    firstHalf = gameInfo.firstHalf == 1;
    secondaryState = (SecondaryState)gameInfo.secondaryState;
    kickOffTeam = (Team)gameInfo.kickOffTeam;
    dropInTeam = (Team)gameInfo.dropInTeam;
    dropInTime = gameInfo.dropInTime;
    secsRemaining = gameInfo.secsRemaining;
    timeSinceLastPackageReceived = int(gameInfo.timeSinceLastPackageReceived);
  }
};

