/**
* @file BH2011LocatorSymbols.h
* Declaration of class BH2011LocatorSymbols.
* @author Judith Müller
*/

#pragma once

#include "Modules/BehaviorControl/Symbols.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Sensing/GroundContactState.h"
#include "Representations/Configuration/FieldDimensions.h"

/**
*
*/
class BH2011LocatorSymbols : public Symbols
{
public:
  /*
  * Constructor.
  */
  BH2011LocatorSymbols(const RobotPose& robotPose, const RobotPoseInfo& robotPoseInfo, const FrameInfo& frameInfo, const GroundContactState& groundContactState, const FieldDimensions& fieldDimensions) :
    robotPose(robotPose),
    robotPoseInfo(robotPoseInfo),
    frameInfo(frameInfo),
    groundContactState(groundContactState),
    fieldDimensions(fieldDimensions),
    lastNoGroundContactSafe(false),
    lastPickupTime(0)
  {}

private:
  float getPoseX();
  float getPoseY();
  float getPoseAngle();
  float getDistanceTo();
  float getAngleTo();
  float getFieldToRelativeX();
  float getFieldToRelativeY();
  float getTimeSinceLastPickup();
  float getTimeSinceLastPoseReset();
  float getGoaliePoseAngle();

  const RobotPose& robotPose;
  const RobotPoseInfo& robotPoseInfo;
  const FrameInfo& frameInfo;
  const GroundContactState& groundContactState;
  const FieldDimensions& fieldDimensions;
  bool lastNoGroundContactSafe;
  unsigned lastPickupTime;

  float angleWidthToOpponentGoal;
  float x; /**< The parameter "x" of the functions "angle_to" and "distance_to" */
  float y; /**< The parameter "y" of the functions "angle_to" and "distance_to" */

  /** registers the symbols at an engine */
  void registerSymbols(xabsl::Engine& engine);

  /** Updates the Locator symbols */
  void update();
};
