#pragma once

#include "Representations/Infrastructure/TeamInfo.h"

struct TeamInfoWrapper
{
  int number;
  GameInfoWrapper::Team color;
  int ownScore,
      oppScore;

  void update(const OwnTeamInfo& own, const OpponentTeamInfo& opp)
  {
    number = own.teamNumber;
    color = (GameInfoWrapper::Team)own.teamColour;
    ownScore = own.score;
    oppScore = opp.score;
  }
};

