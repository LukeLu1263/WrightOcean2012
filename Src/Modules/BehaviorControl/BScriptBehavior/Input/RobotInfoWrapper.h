#pragma once

#include "Representations/Infrastructure/RobotInfo.h"

struct RobotInfoWrapper
{
  enum Penalty
  {
    none = PENALTY_NONE,
    manual = PENALTY_MANUAL,
  };

  void update(const RobotInfo& robotInfo)
  {
    penalty = (Penalty)robotInfo.penalty;
    number = robotInfo.number;
    role = robotInfo.number;
  }

  Penalty penalty;
  int number;
  int role;
};

