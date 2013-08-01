/**
* @file BH2011FallDownSymbols.cpp
*
* Implementation of class FallDownSymbols.
*
* @author Judith Müller
*/

#include "BH2011FallDownSymbols.h"
#include "Tools/Xabsl/B-Human/BHXabslTools.h"
#include <cstdio>

void BH2011FallDownSymbols::registerSymbols(xabsl::Engine& engine)
{
  // The FallDownState
  char s[256];
  engine.registerEnumeratedInputSymbol("fall_down_state", "fall_down_state", (int*)&state.state);
  engine.registerEnumeratedInputSymbol("fall_down_direction", "fall_down_direction", (int*)&state.direction);
  for(int i = 0; i < FallDownState::numOfStates; i++)
  {
    sprintf(s, "fall_down_state.");
    getXabslString(s + strlen(s), FallDownState::getName((FallDownState::State)i));
    engine.registerEnumElement("fall_down_state", s, i);
  }
  for(int i = 0; i < FallDownState::numOfDirections; i++)
  {
    sprintf(s, "fall_down_direction.");
    getXabslString(s + strlen(s), FallDownState::getName((FallDownState::Direction)i));
    engine.registerEnumElement("fall_down_direction", s, i);
  }
}
