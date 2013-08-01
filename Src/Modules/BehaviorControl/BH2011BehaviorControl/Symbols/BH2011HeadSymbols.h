/**
* @file BH2011HeadSymbols.h
* Declaration of class BH2009HeadSymbols.
* @author Colin Graf
*/

#pragma once

#include "Modules/BehaviorControl/Symbols.h"
#include "Representations/MotionControl/HeadMotionRequest.h"
#include "Representations/BehaviorControl/HeadMotionProposal.h"
#include "Representations/MotionControl/HeadJointRequest.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Modeling/RobotPose.h"
#include "Tools/Math/Common.h"

/**
* The Xabsl symbols that are defined in "head_symbols.xabsl"
* @author Max Risler
*/
class BH2011HeadSymbols : public Symbols
{
public:
  /*
  * Constructor.
  * @param headMotionRequest A reference to the HeadMotionRequest.
  */
  BH2011HeadSymbols(HeadMotionRequest& headMotionRequest, HeadMotionProposal& headMotionProposal,const HeadJointRequest& headJointRequest,
	  const FieldDimensions& fieldDimensions, const RobotPose& robotPose);
private:
  void setTilt(float tilt);
  float getTilt();
  void setPan(float pan);
  float getPan();
  void setSpeed(float speed);
  float getSpeed();
  float scanForBallBoundary(); /** Returns the maximum distance to scan for the ball. [mm] */

  HeadMotionProposal& headMotionProposal; /**< A reference to the HeadMotionProposal */
  HeadMotionRequest& headMotionRequest; /**< A reference to the HeadMotionRequest */
  const HeadJointRequest& headJointRequest; /**< A reference to the HeadJointRequest */
  const FieldDimensions& fieldDimensions; /**< A reference to the FieldDimensions */
  const RobotPose& robotPose; /**< A reference to the RobotPose */
  const float sfbIgnoreMargin; /**< Margin which isn't looked at explicitly while scanning for the ball. */
  FieldDimensions::LinesTable sfbBoundary; /**< Boundary for scan for ball. */

  /**
  * Registers the symbols at an engine.
  * @param engine The xabsl engine.
  */
  void registerSymbols(xabsl::Engine& engine);

  /** updates the symbols */
  void update();

  /** Initializes attributes which depend on representations. */
  void init();
};
