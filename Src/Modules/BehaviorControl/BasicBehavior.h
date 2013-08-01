/**
* @file BasicBehavior.h
* Declaration of a BasicBahavior base class
* @author Colin Graf
*/

#pragma once

#include "Tools/Xabsl/XabslEngine/XabslBasicBehavior.h"

/**
* The base class for all basic behaviors
*/
class BasicBehavior : public xabsl::BasicBehavior
{
public:
  /**
  * Constructor
  * @param name The name of the basic behavior
  * @param errorHandler A reference to the error handler
  */
  BasicBehavior(const char* name, xabsl::ErrorHandler& errorHandler) : xabsl::BasicBehavior(name, errorHandler) {}

  /**
  * Virtual destructor
  */
  virtual ~BasicBehavior() {}

  /** Updates the basic behavior.
  * This function is called in each cognition cycle. Use it to register pollable stuff.
  */
  virtual void update() {}

  /** Initializes the basic behavior */
  virtual void init() {}
};
