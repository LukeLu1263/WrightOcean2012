/**
* @file BH2011BehaviorLEDSymbols.h
* Declaration of class BH2011BehaviorLEDSymbols.
* @author jeff
*/

#pragma once

#include "Modules/BehaviorControl/Symbols.h"
#include "Representations/BehaviorControl/BehaviorLEDRequest.h"

class BH2011BehaviorLEDSymbols : public Symbols
{
public:
  /* Constructor */
  BH2011BehaviorLEDSymbols(BehaviorLEDRequest& behaviorLEDRequest)
    : behaviorLEDRequest(behaviorLEDRequest)
  {}

  /** Registers the symbols at an engine */
  void registerSymbols(xabsl::Engine& engine);

  /** Update function, called every cycle */
  void update();

private:
  /** A reference to the BehaviorLEDRequest */
  BehaviorLEDRequest& behaviorLEDRequest;
};

