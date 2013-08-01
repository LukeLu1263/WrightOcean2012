#pragma once

#include "Representations/MotionControl/MotionRequest.h"
#include "../InputRepresentations.h"

class TorsoMatrix;
class RobotDimensions;
class RobotModel;

class BSBikeDynPoints
{
public:
  BSBikeDynPoints(MotionRequest* motionRequest, const InputRepresentations& inputReprs)
    : motionRequest(motionRequest),
      torsoMatrix(inputReprs.theTorsoMatrix),
      robotDimensions(inputReprs.theRobotDimensions),
      robotModel(inputReprs.theRobotModel),
      robotPose(inputReprs.theRobotPose)
  {}

  void set(BikeRequest::BMotionID bmotion, const Vector2<> &ball, bool mirror);

private:
  Vector2<> calculateBallInRobotOrigin(const Vector2<>& ballRel);
  float getDurationPolynomial(float tx, float ty);

  MotionRequest* motionRequest;
  const TorsoMatrix& torsoMatrix;
  const RobotDimensions& robotDimensions;
  const RobotModel& robotModel;
  const RobotPose& robotPose;
};

