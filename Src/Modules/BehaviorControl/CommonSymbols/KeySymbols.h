/**
* @file KeySymbols.h
*
* Declaration of class KeySymbols.
*
* @author Judith Müller
* @author Max Risler
* @author Michael Mester
*/

#pragma once

#include "Modules/BehaviorControl/Symbols.h"
#include "Representations/Infrastructure/KeyStates.h"

class KeySymbols : public Symbols
{
public:
  KeySymbols(KeyStates const& keyStates):
    keyStates(keyStates), leftFoot(false), rightFoot(false)
  {
    for(int i = 0; i < KeyStates::numOfKeys; ++i)
    {
      wasPressed[i] = false;
      pressedAndReleased[i] = false;
    }
  }

  /** registers the symbols at an engine */
  void registerSymbols(xabsl::Engine& engine);
  void update();

private:
  KeyStates const& keyStates;
  bool wasPressed[KeyStates::numOfKeys];
  bool pressedAndReleased[KeyStates::numOfKeys];
  bool leftFoot;
  bool rightFoot;
};
