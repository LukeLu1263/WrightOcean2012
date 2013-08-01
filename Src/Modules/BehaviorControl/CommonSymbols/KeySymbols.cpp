/**
* @file KeySymbols.cpp
*
* Implementation of class KeySymbols.
*
* @author Judith Müller
*/

#include "KeySymbols.h"

void KeySymbols::registerSymbols(xabsl::Engine& engine)
{
  engine.registerBooleanInputSymbol("key.right_foot_pressed_and_released", &rightFoot);
  engine.registerBooleanInputSymbol("key.left_foot_pressed_and_released", &leftFoot);
  engine.registerBooleanInputSymbol("key.chest_button_pressed_and_released", &pressedAndReleased[KeyStates::chest]);
}

void KeySymbols::update()
{
  for(int i = 0; i < KeyStates::numOfKeys; ++i)
  {
    if(wasPressed[i] && !keyStates.pressed[i])
    {
      pressedAndReleased[i] = true;
    }
    else
    {
      pressedAndReleased[i] = false;
    }
    wasPressed[i] = keyStates.pressed[i];
  }
  rightFoot = pressedAndReleased[KeyStates::rightFootRight] || pressedAndReleased[KeyStates::rightFootLeft];
  leftFoot = pressedAndReleased[KeyStates::leftFootRight] || pressedAndReleased[KeyStates::leftFootLeft];
}
