#pragma once

#include "Representations/Infrastructure/KeyStates.h"

struct KeyStatesWrapper
{
  KeyStatesWrapper()
    : chest(false),
      lastChest(false)
  {}

  bool chest;
  bool lastChest;
  bool leftFoot, rightFoot;
  bool lastLeftFoot, lastRightFoot;

  void update(const KeyStates& keyStates)
  {
    chest = lastChest && !keyStates.pressed[KeyStates::chest];
    lastChest = keyStates.pressed[KeyStates::chest];

    bool lf = keyStates.pressed[KeyStates::leftFootLeft] || keyStates.pressed[KeyStates::leftFootRight],
         rf = keyStates.pressed[KeyStates::rightFootLeft] || keyStates.pressed[KeyStates::rightFootRight];

    leftFoot = lastLeftFoot && !lf;
    rightFoot = lastRightFoot && !rf;
    lastLeftFoot = lf;
    lastRightFoot = rf;
  }
};

