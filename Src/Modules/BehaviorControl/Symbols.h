/**
* @file Symbols.h
*
* Declaration of class Symbols.
*
* @author Max Risler
*/

#pragma once

#include "Tools/Xabsl/XabslEngine/XabslEngine.h"
#include "Tools/Global.h"
#include "Tools/Settings.h"

/**
* The base class for all classes which define Xabsl symbols for the BehaviorControl module.
*
* @author Max Risler
*/
class Symbols
{
public:
  /**
  * Virtual destructor.
  */
  virtual ~Symbols() {}

  /** registers the symbols at an engine */
  virtual void registerSymbols(xabsl::Engine& engine) = 0;

  /** updates the symbols */
  virtual void update() {}

  /** initialize the symbols */
  virtual void init() {}
};
