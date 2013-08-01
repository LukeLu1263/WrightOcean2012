/** 
* @file BH2011HeadControlEngine.cpp
* @author Colin Graf
*/

#include "BH2011HeadControlEngine.h"

MAKE_MODULE(BH2011HeadControlEngine, Behavior Control);

BH2011HeadControlEngine::BH2011HeadControlEngine() : lastUsedPointIndex(-1), lastTargetSwitchTime(0) {}

void BH2011HeadControlEngine::init()
{
  p.timeFactor = 1.f;
  p.fieldDistanceFactor = 0.1f;
  p.imageDistanceFactor = 0.1f;
  p.ballFactorBonus = 20.f;

  p.defaultHeadSpeed =  3.4906585f;
  //p.minMovingSpeed = fromDegrees(20);

  int i = 0;

  pointsOfInterest[i++] = PointOfInterest(0, 0, true); // estimate ball position
  pointsOfInterest[i++] = PointOfInterest(0, 0, true); // left from estimate ball
  pointsOfInterest[i++] = PointOfInterest(0, 0, true); // right from estimate ball
  pointsOfInterest[i++] = PointOfInterest(0, 0, true); // last ball percept position
  
  for(int j = 0; j < TeamMateData::numOfPlayers; ++j)
    pointsOfInterest[i++] = PointOfInterest(0, 0);; // leave room for team mate reported ball position

  pointsOfInterest[i++] = PointOfInterest(150, 300, true); // look for ball in front of robot (left)
  pointsOfInterest[i++] = PointOfInterest(150, -300, true); // look for ball in front of robot (right)

  pointsOfInterest[i++] = PointOfInterest(2500, 0, true); // look ahead 

  pointsOfInterest[i++] = PointOfInterest(0, 0, true); // opponent left goal post
  pointsOfInterest[i++] = PointOfInterest(0, 0, true); // opponent right goal post
  
  pointsOfInterest[i++] = PointOfInterest(0, 0, true); // own left goal post
  pointsOfInterest[i++] = PointOfInterest(0, 0, true); // own right goal post
  
  pointsOfInterest[i++] = PointOfInterest(theFieldDimensions.xPosOpponentGoalpost, theFieldDimensions.yPosLeftGoal); // left goal post
  pointsOfInterest[i++] = PointOfInterest(theFieldDimensions.xPosOpponentGoalpost, theFieldDimensions.yPosRightGoal); // right goal post
  pointsOfInterest[i++] = PointOfInterest(theFieldDimensions.xPosOpponentPenaltyArea, theFieldDimensions.yPosLeftPenaltyArea); // left penalty area corner
  pointsOfInterest[i++] = PointOfInterest(theFieldDimensions.xPosOpponentPenaltyArea, theFieldDimensions.yPosRightPenaltyArea); // right penalty area corner
  pointsOfInterest[i++] = PointOfInterest(theFieldDimensions.xPosOpponentPenaltyArea, 0); // point in front of goal

  pointsOfInterest[i++] = PointOfInterest(theFieldDimensions.xPosOwnGoalpost, theFieldDimensions.yPosLeftGoal); // own left goal post
  pointsOfInterest[i++] = PointOfInterest(theFieldDimensions.xPosOwnGoalpost, theFieldDimensions.yPosRightGoal); // own right goal post
  pointsOfInterest[i++] = PointOfInterest(theFieldDimensions.xPosOwnPenaltyArea, theFieldDimensions.yPosLeftPenaltyArea); // own left penalty area corner
  pointsOfInterest[i++] = PointOfInterest(theFieldDimensions.xPosOwnPenaltyArea, theFieldDimensions.yPosRightPenaltyArea); // own right penalty area corner
  pointsOfInterest[i++] = PointOfInterest(theFieldDimensions.xPosOwnPenaltyArea, 0); // own point in front of goal

  pointsOfInterest[i++] = PointOfInterest(0, theFieldDimensions.centerCircleRadius); // left center circle
  pointsOfInterest[i++] = PointOfInterest(0, -theFieldDimensions.centerCircleRadius); // right center circle
  pointsOfInterest[i++] = PointOfInterest(theFieldDimensions.centerCircleRadius, 0); 
  pointsOfInterest[i++] = PointOfInterest(-theFieldDimensions.centerCircleRadius, 0);
  pointsOfInterest[i++] = PointOfInterest(0, 0);

  pointsOfInterest[i++] = PointOfInterest(0, theFieldDimensions.yPosLeftGroundline); // left ground line
  pointsOfInterest[i++] = PointOfInterest(0, theFieldDimensions.yPosRightGroundline); // right ground line

  ASSERT(i == sizeof(pointsOfInterest) / sizeof(*pointsOfInterest));

  //lastIteration = theFrameInfo.time - 100;
  lastUsedPointIndex = -1;
}

void BH2011HeadControlEngine::update(HeadMotionRequest& headMotionRequest)
{
  MODIFY("module:BH2011HeadControlEngine:parameters", p);  
  DECLARE_DEBUG_DRAWING("module:BH2011HeadControlEngine:Image", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:BH2011HeadControlEngine:Field", "drawingOnField");

  // activate or deactivate extra "look for ball" points
  int i = 0;
  bool ballDisappeared = theFrameInfo.getTimeSince(theBallHypotheses.timeWhenDisappeared) > 200 && theHeadMotionProposal.ballFactor > 0.f;

  pointsOfInterest[i].pointOnField = theBallModel.estimate.position;
  pointsOfInterest[i].active = theBallModel.estimate.position != Vector2<>();
  ++i;

  pointsOfInterest[i].pointOnField = theBallModel.estimate.position + Vector2<>(0.f, 300.f);
  pointsOfInterest[i].active = pointsOfInterest[0].active && ballDisappeared;
  ++i;

  pointsOfInterest[i].pointOnField = theBallModel.estimate.position + Vector2<>(0.f, -300.f);
  pointsOfInterest[i].active = pointsOfInterest[0].active && ballDisappeared;
  ++i;

  pointsOfInterest[i].pointOnField = theBallModel.lastPerception.position;
  pointsOfInterest[i].active = theBallModel.lastPerception.position != Vector2<>() && ballDisappeared;
  ++i;
  
  for(int j = 0; j < TeamMateData::numOfPlayers; ++j)
  {
    pointsOfInterest[i].active = ballDisappeared && theFrameInfo.getTimeSince(theTeamMateData.ballModels[j].timeWhenLastSeen) < 1000;
    if(pointsOfInterest[i].active)
      pointsOfInterest[i].pointOnField = theTeamMateData.robotPoses[j] * theTeamMateData.ballModels[j].estimate.position;
    ++i;
  }
  
  pointsOfInterest[i++].active = ballDisappeared;
  pointsOfInterest[i++].active = ballDisappeared;
  pointsOfInterest[i++].active = true;

  const Vector2<int>* pointOnField = &theGoalPercept.posts[theFrameInfo.getTimeSince(theGoalPercept.posts[GoalPercept::LEFT_OPPONENT].timeWhenLastSeen) < 10000 ? GoalPercept::LEFT_OPPONENT : GoalPercept::UNKNOWN_OPPONENT].positionOnField;
  pointsOfInterest[i].pointOnField = Vector2<>(float(pointOnField->x), float(pointOnField->y));
  pointsOfInterest[i].active = theFrameInfo.getTimeSince(theGoalPercept.posts[GoalPercept::LEFT_OPPONENT].timeWhenLastSeen) < 10000 || theFrameInfo.getTimeSince(theGoalPercept.posts[GoalPercept::UNKNOWN_OPPONENT].timeWhenLastSeen) < 10000;
  ++i;
  pointOnField = &theGoalPercept.posts[GoalPercept::RIGHT_OPPONENT].positionOnField;
  pointsOfInterest[i].pointOnField = Vector2<>(float(pointOnField->x), float(pointOnField->y));
  pointsOfInterest[i].active = theFrameInfo.getTimeSince(theGoalPercept.posts[GoalPercept::RIGHT_OPPONENT].timeWhenLastSeen) < 10000;
  ++i;
  pointOnField = &theGoalPercept.posts[theFrameInfo.getTimeSince(theGoalPercept.posts[GoalPercept::LEFT_OWN].timeWhenLastSeen) < 10000 ? GoalPercept::LEFT_OWN : GoalPercept::UNKNOWN_OWN].positionOnField;
  pointsOfInterest[i].pointOnField = Vector2<>(float(pointOnField->x), float(pointOnField->y));
  pointsOfInterest[i].active = theFrameInfo.getTimeSince(theGoalPercept.posts[GoalPercept::LEFT_OWN].timeWhenLastSeen) < 10000 || theFrameInfo.getTimeSince(theGoalPercept.posts[GoalPercept::UNKNOWN_OWN].timeWhenLastSeen) < 10000;
  ++i;
  pointOnField = &theGoalPercept.posts[GoalPercept::RIGHT_OWN].positionOnField;
  pointsOfInterest[i].pointOnField = Vector2<>(float(pointOnField->x), float(pointOnField->y));
  pointsOfInterest[i].active = theFrameInfo.getTimeSince(theGoalPercept.posts[GoalPercept::RIGHT_OWN].timeWhenLastSeen) < 10000;
  ++i;

  // detect head motion
  //const float deltaTime = float(theFrameInfo.time - lastIteration) * 0.001f;
  //Vector2<> measuredPosition(theFilteredJointData.angles[JointData::HeadYaw], theFilteredJointData.angles[JointData::HeadPitch]);
  //bool moving = (measuredPosition - lastMeasuredPosition).squareAbs() > sqr(p.minMovingSpeed * deltaTime);
  bool moving = theHeadJointRequest.moving;

  // find best rated point of interest
  Pose3D cameraMatrixInv = theCameraMatrix.invert();
  Pose2D robotPoseInv = theRobotPose.invert();
  float ballFactor = theHeadMotionProposal.ballFactor > 1.f ? 1.f : theHeadMotionProposal.ballFactor;
  float highestInterest = 0.;
  int bestPointIndex = -1;
  bool targetSeen = false;
  float maxAngle = fromDegrees(119) + (fromDegrees(70) - fromDegrees(119)) * (ballDisappeared ? 0 : ballFactor);
  float ballAngle = fabs(theBallModel.estimate.position.angle()) + 0.01f;
  if(!ballDisappeared && ballAngle > maxAngle)
    maxAngle = ballAngle;
  for(unsigned int i = 0; i < sizeof(pointsOfInterest) / sizeof(*pointsOfInterest); ++i)
  {
    PointOfInterest& point = pointsOfInterest[i];

    Vector2<> relativePoint = point.relative ? point.pointOnField : (robotPoseInv * point.pointOnField);
    float rpAbsAngle = fabs(relativePoint.angle());
    bool reachable = (rpAbsAngle < fromDegrees(80)) || (rpAbsAngle < fromDegrees(119) && (relativePoint.abs() > 1800 || rpAbsAngle > fromDegrees(105)));
    if(reachable && rpAbsAngle > maxAngle)
      reachable = false;
    if(reachable)
      reachable = relative2image(cameraMatrixInv, relativePoint, point.pointInImage);

    COMPLEX_DRAWING("module:BH2011HeadControlEngine:Field",
    {
      if(point.active || int(i) == lastUsedPointIndex)
      {
        Vector2<> point = theRobotPose * relativePoint;
        CIRCLE("module:BH2011HeadControlEngine:Field", point.x, point.y, 30, 20, Drawings::ps_solid, ColorRGBA(0xff, 0, 0, reachable ? 0xff : 0x70), Drawings::bs_null, ColorRGBA());
      }
    });

    COMPLEX_DRAWING("module:BH2011HeadControlEngine:Image",
    {
      if(point.active || int(i) == lastUsedPointIndex)
        if(reachable)
        {
          CIRCLE("module:BH2011HeadControlEngine:Image", theCameraInfo.opticalCenter.x + point.pointInImage.x, theCameraInfo.opticalCenter.y + point.pointInImage.y, 3, 2, Drawings::ps_solid, ColorRGBA(0xff, 0, 0, reachable ? 0xff : 0x70), Drawings::bs_null, ColorRGBA());
        }
    });

    if(!reachable)
    {
      if(int(i) == lastUsedPointIndex)
        targetSeen = true;
      continue;
    }
    
    float sqrPointDistance = point.pointInImage.squareAbs();
    if(sqrPointDistance < sqr(40.f) || // target seen
      (int(i) == lastUsedPointIndex && theFrameInfo.time - lastTargetSwitchTime > 200 && !theHeadJointRequest.reachable && sqrPointDistance < sqr(relativePoint.squareAbs() > sqr(2000.f) ? 100.f : 80.f)) ||
      (int(i) == lastUsedPointIndex && theFrameInfo.time - lastTargetSwitchTime > 200 && !moving)) // avoid head motion stucking
    {
      point.lastSeenTime = theFrameInfo.time;
      if(int(i) == lastUsedPointIndex)
        targetSeen = true;
    }
    else if(point.active || int(i) == lastUsedPointIndex)
    {
      float interest = (theFrameInfo.time - point.lastSeenTime) * (p.timeFactor + 
        relativePoint.squareAbs() * -0.001f * 0.001f * sqr(p.fieldDistanceFactor) + 
        sqrPointDistance * -0.001f * 0.001f * sqr(p.imageDistanceFactor));
      if(i == 0 && !ballDisappeared)
        interest *= (1.f + ballFactor * p.ballFactorBonus);
      //if(ballFactor >= 1.f && ballDisappeared && !point.relative)
        //interest -= 1000000.f; // prefer ball points, when ballFactor is >= 1.f
      if(bestPointIndex == -1 || interest > highestInterest)
      {
        highestInterest = interest;
        bestPointIndex = int(i);
      }
    }
  }
  if(!targetSeen && lastUsedPointIndex != -1)
    bestPointIndex = lastUsedPointIndex;

  // generate head motion request
  if(theHeadMotionProposal.useActiveVision)
  {
    headMotionRequest.mode = HeadMotionRequest::panTiltMode;
    headMotionRequest.speed = theHeadMotionProposal.speed <= 0 ? p.defaultHeadSpeed : theHeadMotionProposal.speed;
    if(bestPointIndex != -1)
    {
      Vector2<>& target = pointsOfInterest[bestPointIndex].pointInImage;
      ARROW("module:BH2011HeadControlEngine:Image", theCameraInfo.opticalCenter.x, theCameraInfo.opticalCenter.y, theCameraInfo.opticalCenter.x + target.x, theCameraInfo.opticalCenter.y + target.y, 2, Drawings::ps_solid, ColorRGBA(0xff, 0, 0));
      headMotionRequest.pan = theFilteredJointData.angles[JointData::HeadYaw] - atan2(target.x, theCameraInfo.focalLength);
      headMotionRequest.tilt = theFilteredJointData.angles[JointData::HeadPitch] - atan2(target.y, theCameraInfo.focalLength);
    }
  }
  else
    headMotionRequest = theHeadMotionProposal;

  // 
  if(bestPointIndex != lastUsedPointIndex)
    lastTargetSwitchTime = theFrameInfo.time;
  lastUsedPointIndex = bestPointIndex;
  //lastIteration = theFrameInfo.time;
  //lastMeasuredPosition = measuredPosition;
}

bool BH2011HeadControlEngine::relative2image(const Pose3D& cameraMatrixInv, const Vector2<>& relative, Vector2<>& image) const
{
  const Vector3<> camera = cameraMatrixInv * Vector3<>(relative.x, relative.y, 0.f);
  if(camera.x > 1.f)
  {
    const float scale = -theCameraInfo.focalLength / camera.x;
    image.x = scale * camera.y;
    image.y = scale * camera.z;
    return true;
  }

  // same calculation (but compatible to larger angles)
  {
    float pan = atan2(relative.y, relative.x);
    float tilt = -atan2(theCameraMatrix.translation.z, relative.abs() - theRobotDimensions.xHeadTiltToCamera) + theRobotDimensions.headTiltToCameraTilt;
    pan = pan - theFilteredJointData.angles[JointData::HeadYaw];
    tilt = tilt - theFilteredJointData.angles[JointData::HeadPitch];
    static const Range<float> pi_2Rang(-pi_2 + 0.1f, pi_2 - 0.1f);
    pan = pi_2Rang.limit(pan);
    tilt = pi_2Rang.limit(tilt);
    image.x = -tan(pan) * theCameraInfo.focalLength;
    image.y = -tan(tilt) * theCameraInfo.focalLength;
    return true;
  }
}
