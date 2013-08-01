/**
* @file BH2011LocatorSymbols.cpp
* Implementation of class BH2011LocatorSymbols.
* @author Judith Müller
* @author Colin Graf
*/

#include "BH2011LocatorSymbols.h"
#include "BH2011BallSymbols.h"

void BH2011LocatorSymbols::registerSymbols(xabsl::Engine& engine)
{
  engine.registerDecimalInputSymbol("locator.pose.x", &robotPose.translation.x);
  engine.registerDecimalInputSymbol("locator.pose.y", &robotPose.translation.y);
  engine.registerDecimalInputSymbol("locator.pose.angle", this, &BH2011LocatorSymbols::getPoseAngle);
  engine.registerDecimalInputSymbol("locator.pose.deviation", &robotPose.deviation);
  engine.registerDecimalInputSymbol("locator.pose.validity", &robotPose.validity);
  engine.registerDecimalInputSymbol("locator.distance_to", this, &BH2011LocatorSymbols::getDistanceTo);
  engine.registerDecimalInputSymbolDecimalParameter("locator.distance_to", "locator.distance_to.x", &x);
  engine.registerDecimalInputSymbolDecimalParameter("locator.distance_to", "locator.distance_to.y", &y);
  engine.registerDecimalInputSymbol("locator.angle_to", this, &BH2011LocatorSymbols::getAngleTo);
  engine.registerDecimalInputSymbolDecimalParameter("locator.angle_to", "locator.angle_to.x", &x);
  engine.registerDecimalInputSymbolDecimalParameter("locator.angle_to", "locator.angle_to.y", &y);
  engine.registerDecimalInputSymbol("locator.field_to_relative.x", this, &BH2011LocatorSymbols::getFieldToRelativeX);
  engine.registerDecimalInputSymbolDecimalParameter("locator.field_to_relative.x", "locator.field_to_relative.x.x", &x);
  engine.registerDecimalInputSymbolDecimalParameter("locator.field_to_relative.x", "locator.field_to_relative.x.y", &y);
  engine.registerDecimalInputSymbol("locator.field_to_relative.y", this, &BH2011LocatorSymbols::getFieldToRelativeY);
  engine.registerDecimalInputSymbolDecimalParameter("locator.field_to_relative.y", "locator.field_to_relative.y.x", &x);
  engine.registerDecimalInputSymbolDecimalParameter("locator.field_to_relative.y", "locator.field_to_relative.y.y", &y);
  engine.registerBooleanInputSymbol("locator.ground_contact", &groundContactState.contact);
  engine.registerDecimalInputSymbol("locator.time_since_last_pickup", this, &BH2011LocatorSymbols::getTimeSinceLastPickup);
  engine.registerDecimalInputSymbol("locator.time_since_last_pose_reset", this, &BH2011LocatorSymbols::getTimeSinceLastPoseReset);
  engine.registerDecimalInputSymbol("locator.opponent_goal.angle_width", &angleWidthToOpponentGoal);

}

void BH2011LocatorSymbols::update()
{
  /** Get time since last pickup */
  if(groundContactState.noContactSafe == false && lastNoGroundContactSafe == true)
    lastPickupTime = frameInfo.time;
  lastNoGroundContactSafe = groundContactState.noContactSafe;
  Pose2D poseForOppGoalAngle = robotPose;
  if(poseForOppGoalAngle.translation.x > float(fieldDimensions.xPosOpponentGroundline - 50))
    poseForOppGoalAngle.translation.x = float(fieldDimensions.xPosOpponentGroundline - 50);
  float angleToLeftOpponentGoalPost  = Geometry::angleTo(poseForOppGoalAngle, Vector2<>((float) fieldDimensions.xPosOpponentGroundline, (float) fieldDimensions.yPosLeftGoal));
  float angleToRightOpponentGoalPost = Geometry::angleTo(poseForOppGoalAngle, Vector2<>((float) fieldDimensions.xPosOpponentGroundline, (float) fieldDimensions.yPosRightGoal));
  angleWidthToOpponentGoal = toDegrees(abs(normalize(angleToLeftOpponentGoalPost - angleToRightOpponentGoalPost) / 2.0f));
}

float BH2011LocatorSymbols::getPoseX()
{
  return robotPose.translation.x;
}

float BH2011LocatorSymbols::getPoseY()
{
  return robotPose.translation.y;
}

float BH2011LocatorSymbols::getPoseAngle()
{
  return toDegrees(robotPose.getAngle());
}

float BH2011LocatorSymbols::getDistanceTo()
{
  return (Pose2D(0.f, x, y) - robotPose).translation.abs();
}

float BH2011LocatorSymbols::getAngleTo()
{
  return toDegrees((Pose2D(0.f, x, y) - robotPose).translation.angle());
}

float BH2011LocatorSymbols::getFieldToRelativeX()
{
  return (robotPose.invert() * Vector2<>(x, y)).x;
}

float BH2011LocatorSymbols::getFieldToRelativeY()
{
  return (robotPose.invert() * Vector2<>(x, y)).y;
}

float BH2011LocatorSymbols::getTimeSinceLastPickup()
{
  return (float)frameInfo.getTimeSince(lastPickupTime);
}

float BH2011LocatorSymbols::getTimeSinceLastPoseReset()
{
  return float(frameInfo.getTimeSince(robotPoseInfo.timeLastPoseReset));
}