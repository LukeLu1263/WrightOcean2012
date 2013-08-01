/**
* @file MathSymbols.h
*
* Declaration of class MathSymbols.
*
* @author Max Risler
*/

#pragma once

#include "Modules/BehaviorControl/Symbols.h"
#include "Platform/SystemCall.h"
#include "Tools/Math/Common.h"

/**
* The Xabsl symbols that are defined in "math_symbols.xabsl"
*
* @author Max Risler
*/
class MathSymbols : public Symbols
{
public:
  /*
  * Constructor.
  */
  MathSymbols() {}

  /** registers the symbols at an engine */
  void registerSymbols(xabsl::Engine& engine);

private:
  /** The parameter "normalize.angle" of the function "normalize" */
  float normalizeAngle;

  /** calculates the decimal input function "sgn" */
  float getSgn();

  /** calculates the decimal input function "min" */
  float getMin();

  /** calculates the decimal input function "max" */
  float getMax();

  /** calculates the decimal input function "abs" */
  float getAbs();

  /** calculates the decimal input function "random" */
  float getRandom();

  /** calculates the decimal input function "sin" */
  float getSin();

  /** calculates the decimal input function "cos" */
  float getCos();

  /** calculates the decimal input function "normalize" */
  float getNormalize();

  /** calculates the decimal input function "clip" */
  float getClip();

  /** calculates the decimal input function "exclude" */
  float getExclude();

  /** calculates the decimal input function "distance" */
  float getDistance();

  /** calculates the decimal input function "distance" */
  float getDistanceAB();

  /** calculates the decimal input function "atan2" */
  float getAtan2();

  /** The parameter "sgn.value" of the function "sgn" */
  float sgnValue;

  /** The parameter "abs.value" of the function "abs" */
  float absValue;

  /** The parameter "min0.value" of the function "min" */
  float min0Value;

  /** The parameter "min1.value" of the function "min" */
  float min1Value;

  /** The parameter "max.value0" of the function "max" */
  float max0Value;

  /** The parameter "max.value1" of the function "max" */
  float max1Value;

  /** The parameter "*.alpha" of the function "sin" and "cos" */
  float alphaValue;

  float clipValue;
  float clipMin;
  float clipMax;

  float excludeValue;
  float excludeMin;
  float excludeMax;

  float distanceX;
  float distanceY;
  float distanceX2;
  float distanceY2;

  float atan2Y;
  float atan2X;

  bool getBetween();
  float valueMin;
  float valueMax;
  float value;
};
