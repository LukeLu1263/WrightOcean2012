/**
* @file BSWalkTo.cpp
* Implementation of the walk_to basic behavior
* @author Colin Graf
*/

#ifndef TARGET_TOOL
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/Modify.h"
#endif

#include "BSWalkTo.h"

BSWalkTo::BSWalkTo(const InputRepresentations& inputRepresentations, MotionRequest* motionRequest)
  : motionRequest(motionRequest),
    theFieldDimensions(inputRepresentations.theFieldDimensions),
    theRobotPose(inputRepresentations.theRobotPose),
    theRobotInfo(inputRepresentations.theRobotInfo),
    theGameInfo(inputRepresentations.theGameInfo),
    theObstacleModel(inputRepresentations.theObstacleModel),
    theRobotsModel(inputRepresentations.theRobotsModel),
    theFrameInfo(inputRepresentations.theFrameInfo),
    theBallModel(inputRepresentations.theBallModel),
    theBallHypotheses(inputRepresentations.theBallHypotheses),
    lastAvoidanceAngleOffset(0.f)
{
  leftPenCorner = Vector2<>(float(theFieldDimensions.xPosOwnPenaltyArea) + p.penaltyAreaExpansion,
                            float(theFieldDimensions.yPosLeftPenaltyArea) + p.penaltyAreaExpansion);
  leftPenGroundline = Vector2<>(float(theFieldDimensions.xPosOwnGroundline) - p.penaltyAreaExpansion,
                                float(theFieldDimensions.yPosLeftPenaltyArea) + p.penaltyAreaExpansion);
  rightPenCorner = Vector2<>(float(theFieldDimensions.xPosOwnPenaltyArea) + p.penaltyAreaExpansion,
                             float(theFieldDimensions.yPosRightPenaltyArea) - p.penaltyAreaExpansion);
  rightPenGroundline = Vector2<>(float(theFieldDimensions.xPosOwnGroundline) - p.penaltyAreaExpansion,
                                 float(theFieldDimensions.yPosRightPenaltyArea) - p.penaltyAreaExpansion);

  leftOppCorner = Vector2<>(float(theFieldDimensions.xPosOpponentGroundline) + p.fieldBorderExpansion,
                            float(theFieldDimensions.yPosLeftGroundline) + p.fieldBorderExpansion);
  rightOppCorner = Vector2<>(float(theFieldDimensions.xPosOpponentGroundline) + p.fieldBorderExpansion,
                             float(theFieldDimensions.yPosRightGroundline) - p.fieldBorderExpansion);
  leftOwnCorner = Vector2<>(float(theFieldDimensions.xPosOwnGroundline) - p.fieldBorderExpansion,
                            float(theFieldDimensions.yPosLeftGroundline) + p.fieldBorderExpansion);
  rightOwnCorner = Vector2<>(float(theFieldDimensions.xPosOwnGroundline) - p.fieldBorderExpansion,
                             float(theFieldDimensions.yPosRightGroundline) - p.fieldBorderExpansion);

  goalPosts[0] = Vector2<>(float(theFieldDimensions.xPosOpponentGoalpost),
                           float(theFieldDimensions.yPosLeftGoal));
  goalPosts[1] = Vector2<>(float(theFieldDimensions.xPosOpponentGoalpost),
                           float(theFieldDimensions.yPosRightGoal));
  goalPosts[2] = Vector2<>(float(theFieldDimensions.xPosOwnGoalpost),
                           float(theFieldDimensions.yPosRightGoal));
  goalPosts[3] = Vector2<>(float(theFieldDimensions.xPosOwnGoalpost),
                           float(theFieldDimensions.yPosLeftGoal));
}

void BSWalkTo::update()
{
#ifndef TARGET_TOOL
  DECLARE_DEBUG_DRAWING("module:BH2011BehaviorControl:BSWalkTo:Field", "drawingOnField");
  MODIFY("module:BH2011BehaviorControl:BSWalkTo:parameters", p);
#endif
}

void BSWalkTo::addObstacle2(const Vector2<>& pos,
                            float expandLength,
                            float avoidanceMinRadius,
                            float avoidanceMaxRadius,
                            float avoidanceDistance)
{
  if(obstacleCount >= sizeof(obstacles) / sizeof(*obstacles))
    return;

  const Vector2<> expand = Vector2<>(-pos.y, pos.x).normalize(expandLength);
  Vector2<> left = pos + expand;
  Vector2<> right = pos - expand;
  const float radius = avoidanceMinRadius + (avoidanceMaxRadius - avoidanceMinRadius) * (avoidanceDistance - pos.abs()) / avoidanceDistance;
  float leftAngle = (left + Vector2<>(-left.y, left.x).normalize(radius)).angle();
  float rightAngle = (right + Vector2<>(right.y, -right.x).normalize(radius)).angle();
  obstacles[obstacleCount++] = Obstacle(left, right, pos, leftAngle, rightAngle);
}
/*
void BSWalkTo::addObstacleWithMaxFactor(const Vector2<>& pos, float expandLength, float avoidanceMinRadius, float avoidanceMaxRadius, float maxFactor)
{
  if(obstacleCount >= sizeof(obstacles) / sizeof(*obstacles))
    return;

  const Vector2<> expand = Vector2<>(-pos.y, pos.x).normalize(expandLength);
  Vector2<> left = pos + expand;
  Vector2<> right = pos - expand;
  const float radius = avoidanceMinRadius + (avoidanceMaxRadius - avoidanceMinRadius) * maxFactor;
  left += Vector2<>(-left.y, left.x).normalize(radius);
  right += Vector2<>(right.y, -right.x).normalize(radius);
  obstacles[obstacleCount++] = Obstacle(left.angle(), right.angle(), pos);
}
*/
void BSWalkTo::addObstacle2(const Vector2<>& left,
                            const Vector2<>& right,
                            const Vector2<>& center,
                            float obstacleDistance,
                            float avoidanceMinRadius,
                            float avoidanceMaxRadius,
                            float avoidanceDistance)
{
  if(obstacleCount >= sizeof(obstacles) / sizeof(*obstacles))
    return;

  const float radius = avoidanceMinRadius + (avoidanceMaxRadius - avoidanceMinRadius) * (avoidanceDistance - obstacleDistance) / avoidanceDistance;
  float leftAngle = (left + Vector2<>(-left.y, left.x).normalize(radius)).angle();
  float rightAngle = (right + Vector2<>(right.y, -right.x).normalize(radius)).angle();
  obstacles[obstacleCount++] = Obstacle(left, right, center, leftAngle, rightAngle);
}

void BSWalkTo::walkTo(Pose2D target, float speed, bool rough)
{
  if(target.translation == Vector2<>())
  {
    generateMotionRequest(target, speed);
    lastAvoidanceAngleOffset = 0;
    return;
  }

  //
  RobotPose robotPoseInv = theRobotPose.invert();
  obstacleCount = 0;

  // the field border is an obstacle
  if(theRobotPose.translation.y > leftOppCorner.y - p.fieldBorderAvoidanceDistance)
  {
    Vector2<> obstaclePos(theRobotPose.translation.x, leftOppCorner.y);
    if(obstaclePos.y < theRobotPose.translation.y + 10.f)
      obstaclePos.y = theRobotPose.translation.y + 10.f;
    obstaclePos = robotPoseInv * obstaclePos;
    addObstacle2(obstaclePos, 300.f, p.fieldBorderAvoidanceMinRadius, p.fieldBorderAvoidanceMaxRadius, p.fieldBorderAvoidanceDistance);
  }
  if(theRobotPose.translation.y < rightOppCorner.y + p.fieldBorderAvoidanceDistance)
  {
    Vector2<> obstaclePos(theRobotPose.translation.x, rightOppCorner.y);
    if(obstaclePos.y > theRobotPose.translation.y - 10.f)
      obstaclePos.y = theRobotPose.translation.y - 10.f;
    obstaclePos = robotPoseInv * obstaclePos;
    addObstacle2(obstaclePos, 300.f, p.fieldBorderAvoidanceMinRadius, p.fieldBorderAvoidanceMaxRadius, p.fieldBorderAvoidanceDistance);
  }
  if(theRobotPose.translation.x > leftOppCorner.x - p.fieldBorderAvoidanceDistance)
  {
    Vector2<> obstaclePos(leftOppCorner.x, theRobotPose.translation.y);
    if(obstaclePos.x < theRobotPose.translation.x + 10.f)
      obstaclePos.x = theRobotPose.translation.x + 10.f;
    obstaclePos = robotPoseInv * obstaclePos;
    addObstacle2(obstaclePos, 300.f, p.fieldBorderAvoidanceMinRadius, p.fieldBorderAvoidanceMaxRadius, p.fieldBorderAvoidanceDistance);
  }
  if(theRobotPose.translation.x < leftOwnCorner.x + p.fieldBorderAvoidanceDistance)
  {
    Vector2<> obstaclePos(leftOwnCorner.x, theRobotPose.translation.y);
    if(obstaclePos.x > theRobotPose.translation.x - 10.f)
      obstaclePos.x = theRobotPose.translation.x - 10.f;
    obstaclePos = robotPoseInv * obstaclePos;
    addObstacle2(obstaclePos, 300.f, p.fieldBorderAvoidanceMinRadius, p.fieldBorderAvoidanceMaxRadius, p.fieldBorderAvoidanceDistance);
  }

  // goal triangles are obstacles
  float sqrFieldBorderAvoidanceDistance = sqr(p.fieldBorderAvoidanceDistance);
  Vector2<> closestPointOnLine;
  if(getSqrDistanceToLine(goalPosts[2], Vector2<>(-1.f, 0.f), 1000.f, theRobotPose.translation, closestPointOnLine) < sqrFieldBorderAvoidanceDistance)
    addObstacle2(closestPointOnLine, 300.f, p.fieldBorderAvoidanceMinRadius, p.fieldBorderAvoidanceMaxRadius, p.fieldBorderAvoidanceDistance);
  if(getSqrDistanceToLine(goalPosts[3], Vector2<>(-1.f, 0.f), 1000.f, theRobotPose.translation, closestPointOnLine) < sqrFieldBorderAvoidanceDistance)
    addObstacle2(closestPointOnLine, 300.f, p.fieldBorderAvoidanceMinRadius, p.fieldBorderAvoidanceMaxRadius, p.fieldBorderAvoidanceDistance);

  // goal posts
  float sqrGoalPostAvoidanceDistance = p.goalPostAvoidanceDistance * p.goalPostAvoidanceDistance;
  for(int i = 0; i < 4; ++i)
    if((goalPosts[i] -  theRobotPose.translation).squareAbs() < sqrGoalPostAvoidanceDistance)
      addObstacle2(robotPoseInv * goalPosts[i], 50.f, p.goalPostAvoidanceMinRadius, p.goalPostAvoidanceMaxRadius, p.goalPostAvoidanceDistance);

  // the own penalty area is an obstacle
  if(theRobotInfo.number != 1 && theGameInfo.state == STATE_PLAYING)
  {
    if(theRobotPose.translation.x < leftPenCorner.x + p.penaltyAreaAvoidanceDistance &&
       theRobotPose.translation.y < leftPenCorner.y + p.penaltyAreaAvoidanceDistance &&
       theRobotPose.translation.y > rightPenCorner.y - p.penaltyAreaAvoidanceDistance) // is near penalty area?
    {
      // compute "obstacle" pos
      const Vector2<> supportPoint(leftPenCorner.x - leftPenCorner.y, 0.f);
      Vector2<> obstacleDir = supportPoint - theRobotPose.translation;
      obstacleDir.normalize(p.penaltyAreaAvoidanceDistance);
      Vector2<> obstaclePos = theRobotPose.translation + obstacleDir;
      if(obstaclePos.x < leftPenCorner.x &&
         obstaclePos.y < leftPenCorner.y &&
         obstaclePos.y > rightPenCorner.y)
      {
        float xoff = (leftPenCorner.x - obstaclePos.x);
        float yoff = (obstaclePos.y > 0.f ? (leftPenCorner.y - obstaclePos.y) : (obstaclePos.y - rightPenCorner.y));
        float a = std::min(xoff, yoff);
        obstacleDir *= std::max(p.penaltyAreaAvoidanceDistance - a, 1.f) * (1.f / p.penaltyAreaAvoidanceDistance);
        if(yoff < xoff)
          obstacleDir = Vector2<>(0.f, obstaclePos.y > 0.f ? -obstacleDir.abs() : obstacleDir.abs());
        else
          obstacleDir = Vector2<>(-obstacleDir.abs(), 0.f);
        obstaclePos = theRobotPose.translation + obstacleDir;
      }
      obstaclePos = robotPoseInv * obstaclePos;
      addObstacle2(obstaclePos, 300.f, p.penaltyAreaAvoidanceMinRadius, p.penaltyAreaAvoidanceMaxRadius, p.penaltyAreaAvoidanceDistance);
    }
  }

  // obstacles from ObstacleModel
  if(true)//!disableObstacleAvoidance)
    for(std::vector<ObstacleModel::Obstacle>::const_iterator it = theObstacleModel.obstacles.begin(), end = theObstacleModel.obstacles.end(); it != end; ++it)
    {
      const ObstacleModel::Obstacle& obstacle = *it;
      const float sqrAbs = obstacle.center.squareAbs();
      if(sqrAbs < sqr(p.obstacleAvoidanceDistance))
        addObstacle2(obstacle.leftCorner, obstacle.rightCorner, obstacle.center, sqrt(sqrAbs), p.obstacleAvoidanceMinRadius, p.obstacleAvoidanceMaxRadius, p.obstacleAvoidanceDistance);
    }

  // obstacles from vision
  if(true)//!disableObstacleAvoidance)
    for(std::vector<RobotsModel::Robot>::const_iterator it = theRobotsModel.robots.begin(), end = theRobotsModel.robots.end(); it != end; ++it)
    {
      const RobotsModel::Robot& robot = *it;
      if(theFrameInfo.getTimeSince(robot.timeStamp) < p.visionObstacleTimeout)
      {
        const float sqrAbs = robot.relPosOnField.squareAbs();
        const float expandLength = robot.standing ? 150.f : 300.f;
        float obstacleAvoidanceDistance = p.obstacleAvoidanceDistance/* + expandLength*/;
        if(sqrAbs < sqr(obstacleAvoidanceDistance))
        {
          //const float obstacleDistance = std::max(sqrt(sqrAbs) - expandLength, 1.f); // reducing the distance causes strange avoidance when an opponent robot is close by
          Vector2<> obstaclePos = robot.relPosOnField;
          //obstaclePos.normalize(obstacleDistance);
          addObstacle2(obstaclePos, expandLength, p.obstacleAvoidanceMinRadius, p.obstacleAvoidanceMaxRadius, p.obstacleAvoidanceDistance);
        }
      }
    }

  // and... the ball is an obstacle
  if(theFrameInfo.getTimeSince(theBallHypotheses.timeWhenDisappeared) < p.ballTimeout)
  {
    const Vector2<>& ballPosition = theBallModel.estimate.position;
    if(ballPosition.squareAbs() < sqr(p.ballAvoidanceDistance))
    {
      // calculate angle between robot and score position (from ball)
      //float alpha = acos((target.translation - ballPosition).normalize() * (-ballPosition).normalize());
      //float radius = p.ballAvoidanceMaxRadius * (alpha / pi);
      addObstacle2(ballPosition, 35.f, p.ballAvoidanceMinRadius, p.ballAvoidanceMaxRadius, p.ballAvoidanceDistance);
    }
  }

  // find out which obstacles disturb our plan
  if(rough)
  {
    float targetSqrDistance = target.translation.squareAbs();
    for(unsigned int i = 0; i < obstacleCount; ++i)
    {
      Obstacle& obstacle = obstacles[i];
      float factor = getOrthogonalProjectionFactor(Vector2<>(), target.translation, obstacle.centerPosition);
      if(factor < 0.f || factor > 1.f || (obstacle.leftPosition.squareAbs() > targetSqrDistance && obstacle.rightPosition.squareAbs() > targetSqrDistance))
        obstacle.active = false;
    }
  }

#ifndef TARGET_TOOL
  COMPLEX_DRAWING("module:BH2011BehaviorControl:BSWalkTo:Field",
  {
    for(unsigned int i = 0; i < obstacleCount; ++i)
    {
      const Obstacle& obstacle = obstacles[i];
      if(!obstacle.active)
        continue;
      Vector2<> base(obstacle.centerPosition.abs(), 0.f);
      base.rotate(obstacle.centerAngle);
      LINE("module:BH2011BehaviorControl:BSWalkTo:Field", 0, 0, base.x, base.y, 0, Drawings::ps_solid, ColorRGBA(0xff, 0, 0));
      for(float j = 0.1f; j < obstacle.openingAngle * 0.5f; j += 0.1f)
      {
        Vector2<> line = base;
        line.rotate(j);
        LINE("module:BH2011BehaviorControl:BSWalkTo:Field", 0, 0, line.x, line.y, 0, Drawings::ps_solid, ColorRGBA(0xaa, 0xaa, 0xaa));
        line = base;
        line.rotate(-j);
        LINE("module:BH2011BehaviorControl:BSWalkTo:Field", 0, 0, line.x, line.y, 0, Drawings::ps_solid, ColorRGBA(0xaa, 0xaa, 0xaa));
      }

      Vector2<> line = base;
      line.rotate(obstacle.openingAngle * 0.5f);
      LINE("module:BH2011BehaviorControl:BSWalkTo:Field", 0, 0, line.x, line.y, 0, Drawings::ps_solid, ColorRGBA(0, 0, 0));
      line = base;
      line.rotate(-obstacle.openingAngle * 0.5f);
      LINE("module:BH2011BehaviorControl:BSWalkTo:Field", 0, 0, line.x, line.y, 0, Drawings::ps_solid, ColorRGBA(0, 0, 0));
    }
  });
#endif

  // test whether the target can be used directly
  float targetAngle = target.translation.angle();
  bool goDirectly = true;
  for(unsigned int i = 0; i < obstacleCount; ++i)
  {
    const Obstacle& obstacle = obstacles[i];
    if(obstacle.active && abs(normalize(obstacle.centerAngle - targetAngle)) < obstacle.openingAngle * 0.5f)
    {
      goDirectly = false;
      break;
    }
  }
  if(goDirectly)
  {
    if(target.translation.x > 200.f)
      target.translation.y = 0.f;
    else if(target.translation.x > 100.f)
      target.translation.y *= 1.f - (target.translation.x - 100.f) / (200.f - 100.f);

    generateMotionRequest(target, speed);
    lastAvoidanceAngleOffset = 0;
    return;
  }

  // find best avoidance angle
  //float minAngle = 0.f, maxAngle = 0.f;
  PossibleAngle possibleAngles[(sizeof(obstacles) / sizeof(*obstacles)) * 2];
  int countOfPossibleAngles = 0;
  for(unsigned int i = 0; i < obstacleCount; ++i)
  {
    const Obstacle& obstacle = obstacles[i];
    if(!obstacle.active)
      continue;
    float angle, angleOffset;
    for(int j = 0; j < 2; ++j)
    {
      if(!(j == 0 ? obstacle.leftIsValid : obstacle.rightIsValid))
        continue;
      angle = obstacle.centerAngle + obstacle.openingAngle * (j == 0 ? 0.5f : -0.5f); // don't normalize this angle
      angleOffset = angle - targetAngle; // same here
      possibleAngles[countOfPossibleAngles].rating = abs(angleOffset) + (sgn(angleOffset) == sgn(lastAvoidanceAngleOffset) ? -pi : 0.f);
      possibleAngles[countOfPossibleAngles].angle = angle;
      possibleAngles[countOfPossibleAngles++].obstacleIndex = i;
    }
  }
  qsort(possibleAngles, countOfPossibleAngles, sizeof(PossibleAngle), (int (*)(const void*, const void*))comparePossibleAngle);

  float avoidanceAngleOffset = 0.f;
  for(int i = 0; i < countOfPossibleAngles; ++i)
  {
    float angle = possibleAngles[i].angle;
    int obstacleIndex = possibleAngles[i].obstacleIndex;
    bool canBeUsed = true;
    for(unsigned int j = 0; j < obstacleCount; ++j)
      if(j != (unsigned int)obstacleIndex)
      {
        const Obstacle& obstacle = obstacles[j];
        if(obstacle.active && abs(normalize(obstacle.centerAngle - angle)) < obstacle.openingAngle * 0.5f)
        {
          canBeUsed = false;
          break;
        }
      }
    if(canBeUsed)
    {
      avoidanceAngleOffset = angle - targetAngle;
      break;
    }
  }
  if(avoidanceAngleOffset == 0.f)
  {
    generateMotionRequest(Pose2D(), speed);
    return;
  }

  if(rough)
    //if(maxAngle - minAngle > pi)
  {
    // avoid warking backwards
    float avoidanceAngle = avoidanceAngleOffset + targetAngle;
    if(avoidanceAngle > pi_2)
      avoidanceAngle = pi_2;
    else if(avoidanceAngle < -pi_2)
      avoidanceAngle = -pi_2;
    avoidanceAngleOffset = avoidanceAngle - targetAngle;
  }

  // generate motion request
  target.translation.rotate(avoidanceAngleOffset);
  //target.rotation = target.translation.angle();
  generateMotionRequest(target, speed);
  lastAvoidanceAngleOffset = avoidanceAngleOffset;
}

void BSWalkTo::generateMotionRequest(const Pose2D& target, float speed)
{
  motionRequest->motion = MotionRequest::walk;
  motionRequest->walkRequest.mode = WalkRequest::targetMode;
  speed /= 100.0f;
  motionRequest->walkRequest.speed = Pose2D(speed, speed, speed);
  motionRequest->walkRequest.target = target;
  motionRequest->walkRequest.dribbling = false;
  motionRequest->walkRequest.kickType = WalkRequest::none;
}

int BSWalkTo::comparePossibleAngle(const PossibleAngle* a, const PossibleAngle* b)
{
  return a->rating > b->rating ? 1 : -1;
}

float BSWalkTo::getOrthogonalProjectionFactor(const Vector2<>& base,
    const Vector2<>& dir,
    const Vector2<>& point) const
{
  Vector2<> dirNorm(dir);
  dirNorm.normalize();
  return ((point.x - base.x) * dirNorm.x + (point.y - base.y) * dirNorm.y) / dir.abs();
}

float BSWalkTo::getSqrDistanceToLine(const Vector2<>& base,
                                     const Vector2<>& dir,
                                     float length,
                                     const Vector2<>& point,
                                     Vector2<>& orthogonalProjection) const
{
  float l = (point.x - base.x) * dir.x + (point.y - base.y) * dir.y;
  if(l < 0)
    l = 0;
  if(l > length)
    l = length;
  orthogonalProjection = base + dir * l;
  return (orthogonalProjection - point).squareAbs();
}

