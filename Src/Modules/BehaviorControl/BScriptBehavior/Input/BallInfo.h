#pragma once

#include "../InputRepresentations.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/BallHypotheses.h"

struct BallInfo
{
  BallState   model,
              vision,
              lastSeenEstimate;
  Vector2<>   endPosition;
  int         lastSeen,
              timeWhenDisappeared,
              timeWhenBallReachesOwnYAxis,
              yPosWhenBallReachesOwnYAxis;

  const MotionInfo* motionInfo;

  void update(const InputRepresentations& inputReps)
  {
    model = inputReps.theBallModel.estimate;
    vision = inputReps.theBallModel.lastPerception;
    lastSeenEstimate = inputReps.theBallModel.lastSeenEstimate;
    endPosition = inputReps.theBallModel.endPosition;
    lastSeen = inputReps.theBallModel.timeWhenLastSeen;
    timeWhenDisappeared = inputReps.theBallHypotheses.timeWhenDisappeared;
    timeWhenBallReachesOwnYAxis = getTimeWhenBallReachesOwnYAxis(inputReps.theFieldDimensions);
    yPosWhenBallReachesOwnYAxis = (int)getYPosWhenBallReachesOwnYAxis();

    motionInfo = &inputReps.theMotionInfo;
  }

  int getTimeWhenBallReachesOwnYAxis(const FieldDimensions& fieldDimensions)
  {
    int result = 1000000;

    float decel = (float)fieldDimensions.ballFriction;
    if(model.velocity.x == 0.0f)
      return result;
    float decelX = model.velocity.x * decel / model.velocity.abs();
    float decelTime = model.velocity.x / decelX;
    float xPosWhenStopping = model.position.x + decelTime * model.velocity.x - 0.5f * decel * sqr(decelTime);
    if(xPosWhenStopping * model.position.x < 0)
    {
      float p(model.velocity.x * 2 / decelX), q(2.0f / decelX * model.position.x);
      float temp =  p * -0.5f + sqrt(sqr(p * 0.5f) - q);
      return int(temp * 1000);
    }
    return int(result * 1000);
  }

  float getYPosWhenBallReachesOwnYAxis()
  {
    if(model.velocity.x == 0 || model.position.x * model.velocity.x > 0) // Ball does not move or moves away
    {
      return 0.0f;
    }
    float timeWhenAxisIsReached = abs(model.position.x / model.velocity.x);
    Vector2<> finalBallPos = model.position + (model.velocity * timeWhenAxisIsReached);

    return finalBallPos.y;
  }

  bool getKickPoseReached(const Pose2D& pose)
  {
    if(!motionInfo->upcomingOdometryOffsetValid)
      return false;
    Pose2D remainingOffset = motionInfo->upcomingOdometryOffset.invert().conc(pose);
    return std::abs(remainingOffset.rotation) < fromDegrees(3.f) && (remainingOffset.translation.squareAbs() < 10.f * 10.f ||
           (remainingOffset.translation.x < 0.f && remainingOffset.translation.x >= -30.f && abs(remainingOffset.translation.y) < 10.f));
  }
};

