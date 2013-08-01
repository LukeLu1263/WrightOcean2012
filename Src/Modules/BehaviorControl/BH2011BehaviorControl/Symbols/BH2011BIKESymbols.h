/**
* @file BH2011BIKESymbols.h
*
* Declaration of class BH2011BIKESymbols.
*
* @author Judith Müller
* @author Tobias Kastner
*/

#pragma once

#include "Modules/BehaviorControl/Symbols.h"
#include "Representations/MotionControl/BikeEngineOutput.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Modeling/RobotPose.h"
#include <vector>

/**
* The Xabsl symbols that are defined in "bike_symbols.xabsl"
*
* @author Judith Müller
*/
class BH2011BIKESymbols : public Symbols
{
public:
  /* Constructor */
  BH2011BIKESymbols(MotionRequest& motionRequest, const FieldDimensions& fieldDimensions, const RobotPose& robotPose) :
    motionRequest(motionRequest), fieldDimensions(fieldDimensions), robotPose(robotPose), updating(false) 
  {}

  MotionRequest& motionRequest; /**< A reference to the MotionRequest */
  const FieldDimensions& fieldDimensions;
  const RobotPose& robotPose;

  /** Registers the symbols at an engine */
  void registerSymbols(xabsl::Engine& engine);
  void update();

private:
  bool getKickForward();

  float ballX, ballY, destAng, strikeOutFactor;
  bool mirror, updating, updates;
  bool kickDuration;
};
