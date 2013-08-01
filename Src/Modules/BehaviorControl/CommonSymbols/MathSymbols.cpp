/**
* @file MathSymbols.cpp
*
* Implementation of class MathSymbols
*
* @author Max Risler
*/

#include "MathSymbols.h"

void MathSymbols::registerSymbols(xabsl::Engine& engine)
{
  // "min"
  engine.registerDecimalInputSymbol("min", this, &MathSymbols::getMin);
  engine.registerDecimalInputSymbolDecimalParameter("min", "min.value0", &min0Value);
  engine.registerDecimalInputSymbolDecimalParameter("min", "min.value1", &min1Value);

  // "max"
  engine.registerDecimalInputSymbol("max", this, &MathSymbols::getMax);
  engine.registerDecimalInputSymbolDecimalParameter("max", "max.value0", &max0Value);
  engine.registerDecimalInputSymbolDecimalParameter("max", "max.value1", &max1Value);

  // "abs"
  engine.registerDecimalInputSymbol("abs", this, &MathSymbols::getAbs);
  engine.registerDecimalInputSymbolDecimalParameter("abs", "abs.value", &absValue);

  // "sgn"
  engine.registerDecimalInputSymbol("sgn", this, &MathSymbols::getSgn);
  engine.registerDecimalInputSymbolDecimalParameter("sgn", "sgn.value", &sgnValue);

  // "sin"
  engine.registerDecimalInputSymbol("sin", this, &MathSymbols::getSin);
  engine.registerDecimalInputSymbolDecimalParameter("sin", "sin.angle", &alphaValue);

  // "cos"
  engine.registerDecimalInputSymbol("cos", this, &MathSymbols::getCos);
  engine.registerDecimalInputSymbolDecimalParameter("cos", "cos.angle", &alphaValue);

  // "random"
  engine.registerDecimalInputSymbol("random", this, &MathSymbols::getRandom);

  // "normalize"
  engine.registerDecimalInputSymbol("normalize", this, &MathSymbols::getNormalize);
  engine.registerDecimalInputSymbolDecimalParameter("normalize", "normalize.angle", &normalizeAngle);

  // "clip"
  engine.registerDecimalInputSymbol("clip", this, &MathSymbols::getClip);
  engine.registerDecimalInputSymbolDecimalParameter("clip", "clip.value", &clipValue);
  engine.registerDecimalInputSymbolDecimalParameter("clip", "clip.min", &clipMin);
  engine.registerDecimalInputSymbolDecimalParameter("clip", "clip.max", &clipMax);

  // "exclude"
  engine.registerDecimalInputSymbol("exclude", this, &MathSymbols::getExclude);
  engine.registerDecimalInputSymbolDecimalParameter("exclude", "exclude.value", &excludeValue);
  engine.registerDecimalInputSymbolDecimalParameter("exclude", "exclude.min", &excludeMin);
  engine.registerDecimalInputSymbolDecimalParameter("exclude", "exclude.max", &excludeMax);

  // "distance"
  engine.registerDecimalInputSymbol("distance", this, &MathSymbols::getDistance);
  engine.registerDecimalInputSymbolDecimalParameter("distance", "distance.x", &distanceX);
  engine.registerDecimalInputSymbolDecimalParameter("distance", "distance.y", &distanceY);

  // "distance"
  engine.registerDecimalInputSymbol("distance_a_to_b", this, &MathSymbols::getDistanceAB);
  engine.registerDecimalInputSymbolDecimalParameter("distance_a_to_b", "distance_a_to_b.x1", &distanceX);
  engine.registerDecimalInputSymbolDecimalParameter("distance_a_to_b", "distance_a_to_b.y1", &distanceY);
  engine.registerDecimalInputSymbolDecimalParameter("distance_a_to_b", "distance_a_to_b.x2", &distanceX2);
  engine.registerDecimalInputSymbolDecimalParameter("distance_a_to_b", "distance_a_to_b.y2", &distanceY2);

  // "atan2" (your friend)
  engine.registerDecimalInputSymbol("atan2", this, &MathSymbols::getAtan2);
  engine.registerDecimalInputSymbolDecimalParameter("atan2", "atan2.y", &atan2Y);
  engine.registerDecimalInputSymbolDecimalParameter("atan2", "atan2.x", &atan2X);

  // "between"
  engine.registerBooleanInputSymbol("between", this, &MathSymbols::getBetween);
  engine.registerBooleanInputSymbolDecimalParameter("between", "between.value", &value);
  engine.registerBooleanInputSymbolDecimalParameter("between", "between.min", &valueMin);
  engine.registerBooleanInputSymbolDecimalParameter("between", "between.max", &valueMax);
}

float MathSymbols::getSgn()
{
  return sgnValue > 0 ? 1.0f : -1.0f;
}

float MathSymbols::getAbs()
{
  return abs(absValue);
}

float MathSymbols::getMin()
{
  if(min0Value < min1Value)
    return min0Value;
  return min1Value;
}

float MathSymbols::getMax()
{
  if(max0Value > max1Value)
    return max0Value;
  return max1Value;
}

float MathSymbols::getSin()
{
  return sin(fromDegrees(alphaValue));
}

float MathSymbols::getCos()
{
  return cos(fromDegrees(alphaValue));
}

float MathSymbols::getRandom()
{
  return ((float) rand() / (RAND_MAX + 1.0f));
}

float MathSymbols::getNormalize()
{
  return toDegrees(normalize(fromDegrees(normalizeAngle)));
}

float MathSymbols::getClip()
{
  if(clipValue < clipMin) return clipMin;
  if(clipValue > clipMax) return clipMax;
  return clipValue;
}

float MathSymbols::getExclude()
{
  if(excludeValue > excludeMin && excludeValue < excludeMax)
    return (excludeMax - excludeValue) <= (excludeValue - excludeMin) ? excludeMax : excludeMin;
  return excludeValue;
}

float MathSymbols::getDistance()
{
  return sqrt(sqr(distanceX) + sqr(distanceY));
}

float MathSymbols::getAtan2()
{
  if(atan2Y == 0.f && atan2X == 0.f)
    return 0.f;
  return toDegrees(atan2(atan2Y, atan2X));
}

float MathSymbols::getDistanceAB()
{
  return sqrt(sqr(distanceX - distanceX2) + sqr(distanceY - distanceY2));
}

bool MathSymbols::getBetween()
{
  return value > valueMin && value < valueMax;
}
