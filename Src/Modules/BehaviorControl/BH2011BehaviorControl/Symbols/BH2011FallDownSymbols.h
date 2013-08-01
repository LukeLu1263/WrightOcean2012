/**
* @file BH2011FallDownSymbols.h
*
* Declaration of class BH2011FallDownSymbols.
*
* @author Judith Müller
*/

#pragma once

#include "Modules/BehaviorControl/Symbols.h"
#include "Representations/Modeling/FallDownState.h"

/**
* The Xabsl symbols that are defined in "fall_down_symbols.xabsl"
*/
class BH2011FallDownSymbols : public Symbols
{
public:
  /*
  * Constructor.
  * @param state A reference to the FallDownState.
  */
  BH2011FallDownSymbols(const FallDownState& state) :
    state(state) {}

  /** A reference to the FallDownState */
  const FallDownState& state;

  /** Registers the symbols at an engine */
  void registerSymbols(xabsl::Engine& engine);
};
