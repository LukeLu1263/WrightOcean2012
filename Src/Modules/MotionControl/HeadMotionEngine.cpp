/**
* @file HeadMotionEngine.cpp
* This file implements a module that creates head joint angles from desired head motion.
* @author <a href="allli@informatik.uni-bremen.de">Alexander Härtl</a>
* @author Colin Graf
*/

#include "HeadMotionEngine.h"
#include "Tools/Math/Common.h"
#include "Tools/Range.h"
#include "Tools/Debugging/DebugDrawings.h" // PLOT

MAKE_MODULE(HeadMotionEngine, Motion Control)

void HeadMotionEngine::init()
{
  lastSpeed = Vector2<>();
  lastIteration = theFrameInfo.time - (unsigned int)(theFrameInfo.cycleTime * 1000.f);

  deathPoints[0] = Geometry::getCircle(
                     Vector2<int>(45, -25), Vector2<int>(120, -20), Vector2<int>(90, -17));
  deathPoints[1] = Geometry::getCircle(
                     Vector2<int>(-45, -25), Vector2<int>(-120, -20), Vector2<int>(-90, -17));
  deathPoints[2] = Geometry::getCircle(
                     Vector2<int>(17, 46), Vector2<int>(85, 23), Vector2<int>(120, 27));
  deathPoints[3] = Geometry::getCircle(
                     Vector2<int>(-17, 46), Vector2<int>(-85, 23), Vector2<int>(-120, 27));

  for(int i = 0; i < 4; ++i)
  {
    deathPoints[i].center.x = fromDegrees(deathPoints[i].center.x);
    deathPoints[i].center.y = fromDegrees(deathPoints[i].center.y);
    deathPoints[i].radius = fromDegrees(deathPoints[i].radius) * 1.015f;
  }
}

void HeadMotionEngine::update(HeadJointRequest& headJointRequest)
{
  const float unclippedPan = theHeadAngleRequest.pan;
  const float unclippedTilt = theHeadAngleRequest.tilt;
  // arbitrary value that seems to be good...
  float maxAcc = 10.f;
  MODIFY("module:HeadMotionEngine:maxAcceleration", maxAcc);

  float pan = unclippedPan == JointData::off ? JointData::off : Range<>(theJointCalibration.joints[JointData::HeadYaw].minAngle, theJointCalibration.joints[JointData::HeadYaw].maxAngle).limit(unclippedPan);
  float tilt = unclippedTilt == JointData::off ? JointData::off : Range<>(theJointCalibration.joints[JointData::HeadPitch].minAngle, theJointCalibration.joints[JointData::HeadPitch].maxAngle).limit(unclippedTilt);

  const float deltaTime = float(theFrameInfo.time - lastIteration) * 0.001f;
  const Vector2<> position(headJointRequest.pan == JointData::off ? theFilteredJointData.angles[JointData::HeadYaw] : headJointRequest.pan,
                           headJointRequest.tilt == JointData::off ? theFilteredJointData.angles[JointData::HeadPitch] : headJointRequest.tilt);
  const Vector2<> target(pan == JointData::off ? 0 : pan, tilt == JointData::off ? 0 : tilt);
  Vector2<> offset(target - position);
  const float distanceToTarget = offset.abs();

  // calculate max speed
  const float maxSpeedForDistance = sqrt(2.f * distanceToTarget * maxAcc * 0.8f);
  const float maxSpeed = min(maxSpeedForDistance, theHeadAngleRequest.speed);

  // max speed clipping
  if(distanceToTarget / deltaTime > maxSpeed)
    offset *= maxSpeed * deltaTime / distanceToTarget; //<=> offset.normalize(maxSpeed * deltaTime);

  // max acceleration clipping
  Vector2<> speed(offset / deltaTime);
  Vector2<> acc((speed - lastSpeed) / deltaTime);
  const float accSquareAbs = acc.squareAbs();
  if(accSquareAbs > maxAcc * maxAcc)
  {
    acc *= maxAcc * deltaTime / sqrt(accSquareAbs);
    speed = acc + lastSpeed;
    offset = speed * deltaTime;
  }
  /* <=>
  Vector2<> speed(offset / deltaTime);
  Vector2<> acc((speed - lastSpeed) / deltaTime);
  if(acc.squareAbs() > maxAcc * maxAcc)
  {
    speed = acc.normalize(maxAcc * deltaTime) + lastSpeed;
    offset = speed * deltaTime;
  }
  */
  PLOT("module:HeadMotionEngine:speed", toDegrees(speed.abs()));

  // calculate new position
  Vector2<> newPosition(position + offset);

  // make sure we don't get to close to the evil points of death
  if(pan != JointData::off && tilt != JointData::off)
    for(int i = 0; i < 4; ++i)
    {
      Vector2<> deathPointToPosition(newPosition - deathPoints[i].center);
      const float deathPointToPositionSquareAbs = deathPointToPosition.squareAbs();
      if(deathPointToPositionSquareAbs < sqr(deathPoints[i].radius))
      {
        const float deathPointToPositionAbs = sqrt(deathPointToPositionSquareAbs);
        deathPointToPosition *= (deathPoints[i].radius - deathPointToPositionAbs) / deathPointToPositionAbs;
        newPosition += deathPointToPosition;
      }
    }

  // set new position
  headJointRequest.pan = pan == JointData::off ? JointData::off : newPosition.x;
  headJointRequest.tilt = tilt == JointData::off ? JointData::off : newPosition.y;
  headJointRequest.moving = pan != JointData::off && tilt != JointData::off && ((newPosition - position) / deltaTime).squareAbs() > sqr(maxAcc * deltaTime * 0.5f);

  // check reachability
  headJointRequest.reachable = true;
  if(pan != unclippedPan || tilt != unclippedTilt)
    headJointRequest.reachable = false;
  else
    for(int i = 0; i < 4; ++i)
      if((target - deathPoints[i].center).squareAbs() < sqr(deathPoints[i].radius))
        headJointRequest.reachable = false;

  // store some values for the next iteration
  lastSpeed = speed;
  lastIteration = theFrameInfo.time;
}
