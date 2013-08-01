/**
* @file CommonBIKESymbols.h
*
* Declaration of class BH2010StableBIKESymbols.
*
* @author Judith Müller
*/

#pragma once

#include "Modules/BehaviorControl/Symbols.h"
#include "Modules/MotionControl/BIKEParameters.h"
#include "Representations/MotionControl/BikeEngineOutput.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Tools/Xabsl/B-Human/BHXabslTools.h"

/**
*
* @author Judith Müller
*/
class CommonBIKESymbols : public Symbols
{
public:
  /* Constructor */
  CommonBIKESymbols(MotionRequest& motionRequest, const BikeEngineOutput& bikeEngineOutput, const MotionInfo& motionInfo) :
    motionRequest(motionRequest),
    bikeEngineOutput(bikeEngineOutput),
    motionInfo(motionInfo)
  {}

  MotionRequest& motionRequest; /**< A reference to the MotionRequest */
  const BikeEngineOutput& bikeEngineOutput; /**< A reference to the bikeEngineOutput (to get when a bmotion is leavable) */
  const MotionInfo& motionInfo;

  /** Registers the symbols at an engine */
  void registerSymbols(xabsl::Engine& engine);

  /** Update function, called every cycle */
  void update() {};

private:
  void setKick(int kick);
  int getKick();
};
