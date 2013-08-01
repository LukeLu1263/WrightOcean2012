/**
* @file SoundSymbols.h
*
* Declaration of class SoundSymbols.
*
* @author Judith Müller
*/

#pragma once

#include "Modules/BehaviorControl/Symbols.h"
#include "Representations/Infrastructure/SoundRequest.h"

class SoundSymbols : public Symbols
{
public:
  /** Constructor.*/
  SoundSymbols(SoundRequest& soundRequest) :
    soundRequest(soundRequest)
  {}

  SoundRequest& soundRequest;

  /** registers the symbols at an engine */
  void registerSymbols(xabsl::Engine& engine);
};
