/**
* @file BH2011HeadSymbols.cpp
* Implementation of class BH2011HeadSymbols.
* @author Colin Graf
*/

#include "BH2011HeadSymbols.h"

BH2011HeadSymbols::BH2011HeadSymbols(HeadMotionRequest& headMotionRequest, HeadMotionProposal& headMotionProposal,
									const HeadJointRequest& headJointRequest,
                                    const FieldDimensions& fieldDimensions, const RobotPose& robotPose) :
  headMotionRequest(headMotionRequest),
  headMotionProposal(headMotionProposal),
  headJointRequest(headJointRequest),
  fieldDimensions(fieldDimensions),
  robotPose(robotPose),
  sfbIgnoreMargin(300)
{}


void BH2011HeadSymbols::registerSymbols(xabsl::Engine& engine)
{
  engine.registerEnumElement("head.mode", "head.mode.pan_tilt", HeadMotionRequest::panTiltMode);
  engine.registerEnumElement("head.mode", "head.mode._target", HeadMotionRequest::targetMode);
  engine.registerEnumElement("head.mode", "head.mode.target_on_ground", HeadMotionRequest::targetOnGroundMode);
  engine.registerEnumeratedOutputSymbol("head.mode", "head.mode", (int*)&headMotionRequest.mode);

  engine.registerEnumElement("head.ccmode", "head.ccmode.autoCamera", HeadMotionRequest::autoCamera);
  engine.registerEnumElement("head.ccmode", "head.ccmode.lowerCamera", HeadMotionRequest::lowerCamera);
  engine.registerEnumElement("head.ccmode", "head.ccmode.upperCamera", HeadMotionRequest::upperCamera);
  engine.registerEnumeratedOutputSymbol("head.ccmode", "head.ccmode", (int*)&headMotionRequest.cameraControlMode);

  engine.registerDecimalOutputSymbol("head.tilt",
                                     this, &BH2011HeadSymbols::setTilt, &BH2011HeadSymbols::getTilt);
  engine.registerDecimalOutputSymbol("head.pan",
                                     this, &BH2011HeadSymbols::setPan, &BH2011HeadSymbols::getPan);
  engine.registerDecimalOutputSymbol("head.speed",
                                     this, &BH2011HeadSymbols::setSpeed, &BH2011HeadSymbols::getSpeed);
  engine.registerDecimalOutputSymbol("head.target.x", &headMotionRequest.target.x);
  engine.registerDecimalOutputSymbol("head.target.y", &headMotionRequest.target.y);
  engine.registerDecimalOutputSymbol("head.target.z", &headMotionRequest.target.z);
  engine.registerBooleanInputSymbol("head.is_moving", &headJointRequest.moving);
  engine.registerBooleanInputSymbol("head.is_reachable", &headJointRequest.reachable);
  engine.registerDecimalInputSymbol("head.sfb_boundary", this, &BH2011HeadSymbols::scanForBallBoundary);
  engine.registerBooleanOutputSymbol("head.use_active_vision", &headMotionProposal.useActiveVision);
  engine.registerDecimalOutputSymbol("head.active_vision_ball_factor", &headMotionProposal.ballFactor);
}

void BH2011HeadSymbols::update()
{
  headMotionRequest.cameraControlMode = HeadMotionRequest::lowerCamera;
  headMotionProposal.useActiveVision = false;
}

void BH2011HeadSymbols::setTilt(float tilt)
{
  headMotionRequest.tilt = fromDegrees(tilt);
}

float BH2011HeadSymbols::getTilt()
{
  return toDegrees(headMotionRequest.tilt);
}

void BH2011HeadSymbols::setPan(float pan)
{
  headMotionRequest.pan = fromDegrees(pan);
}

float BH2011HeadSymbols::getPan()
{
  return toDegrees(headMotionRequest.pan);
}

void BH2011HeadSymbols::setSpeed(float speed)
{
  headMotionRequest.speed = fromDegrees(speed);
}

float BH2011HeadSymbols::getSpeed()
{
  return toDegrees(headMotionRequest.speed);
}

float BH2011HeadSymbols::scanForBallBoundary()
{
  return sfbBoundary.getDistance(robotPose);
}

void BH2011HeadSymbols::init()
{
  const float bottom = fieldDimensions.xPosOwnGroundline + sfbIgnoreMargin;
  const float top = fieldDimensions.xPosOpponentGroundline - sfbIgnoreMargin;
  const float left = fieldDimensions.yPosLeftSideline - sfbIgnoreMargin;
  const float right = fieldDimensions.yPosRightSideline + sfbIgnoreMargin;

  const Vector2<> bottomLeft(bottom, left);
  const Vector2<> bottomRight(bottom, right);
  const Vector2<> topLeft(top, left);
  const Vector2<> topRight(top, right);

  sfbBoundary.push(bottomLeft, bottomRight, false);
  sfbBoundary.push(bottomRight, topRight, false);
  sfbBoundary.push(topRight, topLeft, false);
  sfbBoundary.push(topLeft, bottomLeft, false);
}
