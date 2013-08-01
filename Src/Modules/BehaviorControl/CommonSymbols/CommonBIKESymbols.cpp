/**
* @file CommonBIKESymbols.cpp
*
* Implementation of class CommonBIKESymbols.
*
* @author Judith Müller
*/

#include <cstdio>

#include "CommonBIKESymbols.h"

void CommonBIKESymbols::registerSymbols(xabsl::Engine& engine)
{
  char s[256];
  char* dest;

  dest = s + sprintf(s, "motion.bike.");
  for(int i = 0; i < BikeRequest::numOfBMotionIDs; ++i)
  {
    getXabslString(dest, BikeRequest::getName((BikeRequest::BMotionID)i));
    engine.registerEnumElement("motion.bike", s, i);
  }
  engine.registerEnumeratedOutputSymbol("motion.bike", "motion.bike",
                                        this, &CommonBIKESymbols::setKick, &CommonBIKESymbols::getKick);

  engine.registerBooleanOutputSymbol("motion.bike.mirror", &motionRequest.bikeRequest.mirror);
  engine.registerBooleanOutputSymbol("motion.bike.dynamical", &motionRequest.bikeRequest.dynamical);

  engine.registerEnumeratedInputSymbol("executed_motion.bike", "motion.bike", (const int*)&motionInfo.bikeRequest.bMotionType);
  engine.registerBooleanInputSymbol("executed_motion.bike.mirror", &motionInfo.bikeRequest.mirror);

  engine.registerBooleanInputSymbol("motion.bike.is_leaving_possible", &bikeEngineOutput.isLeavingPossible);
}

void CommonBIKESymbols::setKick(int kick)
{
  motionRequest.bikeRequest.bMotionType = BikeRequest::BMotionID(kick);
}

int CommonBIKESymbols::getKick()
{
  return int(motionRequest.bikeRequest.bMotionType);
}


