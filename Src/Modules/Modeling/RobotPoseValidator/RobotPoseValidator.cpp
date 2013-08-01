/**
* @file RobotPoseValidator.cpp
* The Implementation of the module that validates and improves robot poses from the self location.
* @author Colin Graf
*/

#include "RobotPoseValidator.h"
#include "Tools/Team.h"

#define COVARIANCE2D(id, cov, mean) \
  COMPLEX_DRAWING(id, \
  { \
    const float factor = 1.f; \
    const float cov012 = cov[0][1] * cov[0][1]; \
    const float varianceDiff = cov[0][0] - cov[1][1]; \
    const float varianceDiff2 = varianceDiff * varianceDiff; \
    const float varianceSum = cov[0][0] + cov[1][1]; \
    const float root = sqrt(varianceDiff2 + 4.0f * cov012); \
    const float eigenValue1 = 0.5f * (varianceSum + root); \
    const float eigenValue2 = 0.5f * (varianceSum - root); \
    \
    const float axis1 = 2.0f * sqrt(factor * eigenValue1); \
    const float axis2 = 2.0f * sqrt(factor * eigenValue2); \
    const float angle = 0.5f * atan2(2.0f * cov[0][1], varianceDiff); \
    \
    ELLIPSE(id, mean, sqrt(3.0f) * axis1, sqrt(3.0f) * axis2, angle, \
            10, Drawings::ps_solid, ColorRGBA(100,100,255,100), Drawings::bs_solid, ColorRGBA(100,100,255,100)); \
    ELLIPSE(id, mean, sqrt(2.0f) * axis1, sqrt(2.0f) * axis2, angle, \
            10, Drawings::ps_solid, ColorRGBA(150,150,100,100), Drawings::bs_solid, ColorRGBA(150,150,100,100)); \
    ELLIPSE(id, mean, axis1, axis2, angle, \
            10, Drawings::ps_solid, ColorRGBA(255,100,100,100), Drawings::bs_solid, ColorRGBA(255,100,100,100)); \
  });


MAKE_MODULE(RobotPoseValidator, Modeling)

RobotPoseValidator::RobotPoseValidator() :
  translationCov(10000.f * 10000.f), rotation(0), rotationCov(pi* pi), lastResetTime(0), validated(false), validGoalSightingSinceLastReset(0), validUnknownGoalSightingSinceLastReset(0)
{
  p.respawnMaxPoseDistance = Pose2D(0.6f, 1000.f, 1000.f);
  p.respawnPoseDeviation = Pose2D(0.6f, 1000.f, 1000.f);
  p.lineRelationCorridor = 300.f;
  p.odometryDeviation = Pose2D(0.3f, 0.2f, 0.2f);
  p.odometryRotationDeviation = Vector2<>(pi_2 / 1000.f, pi_2 / 1000.f);
  p.filterProcessDeviation = Pose2D(0.001f, 0.1f, 0.1f);
  p.robotRotationDeviation = Vector2<>(0.02f, 0.06f);
  p.validationMaxDeviation = Pose2D(0.1f, 100.f, 100.f);
  p.validationMaxGoalPerceptDistance = 0.1f;
  p.validationMinGoalSightings = 6;
  p.validationMinUnknownGoalSightings = 10;
}

void RobotPoseValidator::init()
{
  // prepare relevant field line table
  countOfFieldLines = 0;
  for(unsigned int i = 0, count = theFieldDimensions.fieldLines.lines.size(); i < count; ++i)
  {
    const FieldDimensions::LinesTable::Line& fieldLine = theFieldDimensions.fieldLines.lines[i];
    if(!fieldLine.isPartOfCircle && fieldLine.length > 300.f)
    {
      ASSERT(countOfFieldLines < int(sizeof(fieldLines) / sizeof(*fieldLines)));
      FieldLine& relevantFieldLine = fieldLines[countOfFieldLines++];
      relevantFieldLine.start = fieldLine.corner.translation;
      relevantFieldLine.end = fieldLine.corner * Vector2<>(fieldLine.length, 0);
      relevantFieldLine.dir = relevantFieldLine.end - relevantFieldLine.start;
      relevantFieldLine.dir.normalize();
      relevantFieldLine.length = fieldLine.length;
      relevantFieldLine.vertical = abs(fieldLine.corner.rotation) < 0.001f || abs(normalize(fieldLine.corner.rotation - pi)) < 0.001f;
    }
  }

  // and goal posts
  ASSERT(GoalPercept::NUMBER_OF_GOAL_POSTS == 4);
  goalPosts[GoalPercept::LEFT_OPPONENT] = Vector2<>(float(theFieldDimensions.xPosOpponentGoalpost), float(theFieldDimensions.yPosLeftGoal));
  goalPosts[GoalPercept::RIGHT_OPPONENT] = Vector2<>(float(theFieldDimensions.xPosOpponentGoalpost), float(theFieldDimensions.yPosRightGoal));
  goalPosts[GoalPercept::LEFT_OWN] = Vector2<>(float(theFieldDimensions.xPosOwnGoalpost), float(theFieldDimensions.yPosRightGoal));
  goalPosts[GoalPercept::RIGHT_OWN] = Vector2<>(float(theFieldDimensions.xPosOwnGoalpost), float(theFieldDimensions.yPosLeftGoal));

  lastResetTime = theFrameInfo.time;
}

void RobotPoseValidator::update(RobotPose& robotPose)
{
  MODIFY("module:RobotPoseValidator:parameters", p);

  DECLARE_DEBUG_DRAWING("module:RobotPoseValidator:image", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:RobotPoseValidator:field", "drawingOnField");
  DECLARE_DEBUG_DRAWING("module:RobotPoseValidator:fieldRel", "drawingOnField");

  motionUpdate();

  // respawn?
  if(abs(translation.x - thePotentialRobotPose.translation.x) > p.respawnMaxPoseDistance.translation.x ||
     abs(translation.y - thePotentialRobotPose.translation.y) > p.respawnMaxPoseDistance.translation.y ||
     abs(normalize(rotation - thePotentialRobotPose.rotation)) > p.respawnMaxPoseDistance.rotation ||
     (!theGroundContactState.contact && theDamageConfiguration.useGroundContactDetection) ||
     theFallDownState.state != theFallDownState.upright)
  {
    translation = Vector<2>(thePotentialRobotPose.translation.x, thePotentialRobotPose.translation.y);
    translationCov = Matrix<2, 2>();
    translationCov[0][0] = sqr(p.respawnPoseDeviation.translation.x);
    translationCov[1][1] = sqr(p.respawnPoseDeviation.translation.y);
    rotation = thePotentialRobotPose.rotation;
    rotationCov = sqr(p.respawnPoseDeviation.rotation);
    bool wasValidated = validated;
    validated = false;
    validGoalSightingSinceLastReset = 0;
    validUnknownGoalSightingSinceLastReset = 0;
    if(wasValidated)
      lastResetTime = theFrameInfo.time;
  }

  // try to relate the line percepts
  Vector2<> intersection, orthogonalProjection;
  float sqrLineRelationCorridor = sqr(p.lineRelationCorridor);
  LinePercept linePercept;
  for(std::list< ::LinePercept::Line>::const_iterator it = theLinePercept.lines.begin(), lastIt = theLinePercept.lines.end(); it != lastIt; ++it)
  {
    const ::LinePercept::Line& line = *it;
    linePercept.start = Vector2<>(float(line.first.x), float(line.first.y));
    linePercept.end = Vector2<>(float(line.last.x), float(line.last.y));
    Vector2<> startOnField = thePotentialRobotPose * linePercept.start;
    Vector2<> endOnField = thePotentialRobotPose * linePercept.end;
    Vector2<> dirOnField = endOnField - startOnField;
    dirOnField.normalize();
    Vector2<> orthogonalOnField(dirOnField.y, -dirOnField.x);

    COMPLEX_DRAWING("module:RobotPoseValidator:field",
    {
      Vector2<> checkLineStart1 = startOnField + orthogonalOnField* p.lineRelationCorridor;
      Vector2<> checkLineEnd1 = startOnField - orthogonalOnField* p.lineRelationCorridor;
      Vector2<> checkLineStart2 = endOnField + orthogonalOnField* p.lineRelationCorridor;
      Vector2<> checkLineEnd2 = endOnField - orthogonalOnField* p.lineRelationCorridor;
      LINE("module:RobotPoseValidator:field", checkLineStart1.x, checkLineStart1.y, checkLineEnd1.x, checkLineEnd1.y, 0, Drawings::ps_solid, ColorRGBA(0xff, 0, 0));
      LINE("module:RobotPoseValidator:field", checkLineStart2.x, checkLineStart2.y, checkLineEnd2.x, checkLineEnd2.y, 0, Drawings::ps_solid, ColorRGBA(0xff, 0, 0));
    });

    int index = -1;
    for(int i = 0; i < countOfFieldLines; ++i)
    {
      const FieldLine& fieldLine = fieldLines[i];

      if(getSqrDistanceToLine(fieldLine.start, fieldLine.dir, fieldLine.length, startOnField) > sqrLineRelationCorridor ||
         getSqrDistanceToLine(fieldLine.start, fieldLine.dir, fieldLine.length, endOnField) > sqrLineRelationCorridor)
        continue;
      if(!intersectLineWithLine(startOnField, orthogonalOnField, fieldLine.start, fieldLine.dir, intersection))
        continue;
      if(getSqrDistanceToLine(startOnField, dirOnField, intersection) > sqrLineRelationCorridor)
        continue;
      if(!intersectLineWithLine(endOnField, orthogonalOnField, fieldLine.start, fieldLine.dir, intersection))
        continue;
      if(getSqrDistanceToLine(startOnField, dirOnField, intersection) > sqrLineRelationCorridor)
        continue;
      if(index != -1)
      {
        // ambiguous?
        index = -1;
        break;
      }
      index = i;
    }

    if(index != -1) // success
      useLinePercept(linePercept, fieldLines[index]);
  }

  // try to relate the goal posts
  for(int i = 0; i < GoalPercept::NUMBER_OF_GOAL_POSTS; ++i)
  {
    const GoalPost& post = theGoalPercept.posts[i];
    if(post.timeWhenLastSeen == theFrameInfo.time)
    {
      Vector2<> position(float(post.positionOnField.x), float(post.positionOnField.y));
      Vector2<> positionOnField = thePotentialRobotPose * position;
      if((positionOnField - goalPosts[i]).squareAbs() < sqrLineRelationCorridor)
        useGoalPost(position, goalPosts[i]);
    }
  }
  for(int i = 0; i < GoalPercept::NUMBER_OF_UNKNOWN_GOAL_POSTS; ++i)
  {
    const GoalPost& post = theGoalPercept.unknownPosts[i];
    if(post.timeWhenLastSeen == theFrameInfo.time)
    {
      Vector2<> position(float(post.positionOnField.x), float(post.positionOnField.y));
      Vector2<> positionOnField = thePotentialRobotPose * position;
      ASSERT(GoalPercept::LEFT_OPPONENT == GoalPercept::RIGHT_OPPONENT - 1);
      ASSERT(GoalPercept::LEFT_OWN == GoalPercept::RIGHT_OWN - 1);
      int index = -1;
      for(int j = i == GoalPercept::UNKNOWN_OWN ? GoalPercept::LEFT_OWN : GoalPercept::LEFT_OPPONENT, end = j + 2; j < end; ++j)
      {
        if((positionOnField - goalPosts[j]).squareAbs() > sqrLineRelationCorridor)
          continue;
        if(index != -1)
        {
          // ambiguous?
          index = -1;
          break;
        }
        index = j;
      }
      if(index != -1)
        useGoalPost(position, goalPosts[index]);
    }
  }

  // try to use the center circle
  if(theLinePercept.circle.found)
  {
    Vector2<> circlePos(float(theLinePercept.circle.pos.x), float(theLinePercept.circle.pos.y));
    Vector2<> circlePosOnField = thePotentialRobotPose * circlePos;
    if(circlePosOnField.squareAbs() < sqrLineRelationCorridor)
      useCenterCircle(circlePos);
  }

  // try to validate the pose via goal posts
  Pose2D ownPose(rotation, translation.x, translation.y);
  if(!validated)
  {
    for(int i = 0; i < GoalPercept::NUMBER_OF_GOAL_POSTS; ++i)
    {
      const GoalPost& post = theGoalPercept.posts[i];
      if(post.timeWhenLastSeen == theFrameInfo.time)
      {
        Vector2<> seenPositionOnField = ownPose * Vector2<>(float(post.positionOnField.x), float(post.positionOnField.y));
        const Vector2<>& realPositionOnField = goalPosts[i];

        if(abs(realPositionOnField.x - translation.x) > float(theFieldDimensions.xPosOpponentGroundline - theFieldDimensions.xPosOpponentPenaltyArea) &&
           (seenPositionOnField - realPositionOnField).squareAbs() / (realPositionOnField - Vector2<>(translation.x, translation.y)).squareAbs() < sqr(p.validationMaxGoalPerceptDistance))
          validGoalSightingSinceLastReset++;
      }
    }
    for(int i = 0; i < GoalPercept::NUMBER_OF_UNKNOWN_GOAL_POSTS; ++i)
    {
      const GoalPost& post = theGoalPercept.unknownPosts[i];
      if(post.timeWhenLastSeen == theFrameInfo.time)
      {
        Vector2<> seenPositionOnField = ownPose * Vector2<>(float(post.positionOnField.x), float(post.positionOnField.y));
        const Vector2<>& realPositionOnField1 = goalPosts[i == GoalPercept::UNKNOWN_OPPONENT ? GoalPercept::LEFT_OPPONENT : GoalPercept::LEFT_OWN];
        const Vector2<>& realPositionOnField2 = goalPosts[i == GoalPercept::UNKNOWN_OPPONENT ? GoalPercept::RIGHT_OPPONENT : GoalPercept::RIGHT_OWN];
        if(abs(realPositionOnField1.x - translation.x) > float(theFieldDimensions.xPosOpponentGroundline - theFieldDimensions.xPosOpponentPenaltyArea) &&
           (seenPositionOnField - realPositionOnField1).squareAbs() / (realPositionOnField1 - Vector2<>(translation.x, translation.y)).squareAbs() < sqr(p.validationMaxGoalPerceptDistance))
          validUnknownGoalSightingSinceLastReset++;
        if(abs(realPositionOnField2.x - translation.x) > float(theFieldDimensions.xPosOpponentGroundline - theFieldDimensions.xPosOpponentPenaltyArea) &&
           (seenPositionOnField - realPositionOnField2).squareAbs() / (realPositionOnField2 - Vector2<>(translation.x, translation.y)).squareAbs() < sqr(p.validationMaxGoalPerceptDistance))
          validUnknownGoalSightingSinceLastReset++;
      }
    }
    if((validGoalSightingSinceLastReset >= p.validationMinGoalSightings || validUnknownGoalSightingSinceLastReset >= p.validationMinUnknownGoalSightings) &&
       translationCov[0][0] <= sqr(p.validationMaxDeviation.translation.x) &&
       translationCov[1][1] <= sqr(p.validationMaxDeviation.translation.y) &&
       rotationCov <= sqr(p.validationMaxDeviation.rotation))
      validated = true;
  }

  // generate model
  robotPose.ownTeamColorForDrawing = theOwnTeamInfo.teamColor == TEAM_BLUE ? ColorRGBA(0, 0, 255) : ColorRGBA(255, 0, 0);
  if(validated)
  {
    robotPose.rotation = ownPose.rotation;
    robotPose.translation = ownPose.translation;
    robotPose.validity = 1.f;
    robotPose.deviation = sqrt(std::max(translationCov[0].x, translationCov[1].y));
  }
  else
  {
    robotPose.rotation = thePotentialRobotPose.rotation;
    robotPose.translation = thePotentialRobotPose.translation;
    robotPose.validity = thePotentialRobotPose.validity;
    robotPose.deviation = RobotPose::unknownDeviation;
  }

  COVARIANCE2D("module:RobotPoseValidator:field", translationCov, translation);

  TEAM_OUTPUT_FAST(idTeamMateRobotPose, bin, RobotPoseCompressed(robotPose));
}

void RobotPoseValidator::update(RobotPoseInfo& robotPoseInfo)
{
  robotPoseInfo.timeLastPoseReset = lastResetTime;
}

bool RobotPoseValidator::intersectLineWithLine(const Vector2<>& lineBase1, const Vector2<>& lineDir1,
    const Vector2<>& lineBase2, const Vector2<>& lineDir2, Vector2<>& intersection) const
{
  float h = lineDir1.x * lineDir2.y - lineDir1.y * lineDir2.x;
  if(h == 0.f)
    return false;
  float scale = ((lineBase2.x - lineBase1.x) * lineDir1.y - (lineBase2.y - lineBase1.y) * lineDir1.x) / h;
  intersection.x = lineBase2.x + lineDir2.x * scale;
  intersection.y = lineBase2.y + lineDir2.y * scale;
  return true;
}

float RobotPoseValidator::getSqrDistanceToLine(const Vector2<>& base, const Vector2<>& dir, float length, const Vector2<>& point) const
{
  float l = (point.x - base.x) * dir.x + (point.y - base.y) * dir.y;
  if(l < 0)
    l = 0;
  if(l > length)
    l = length;
  return ((base + dir * l) - point).squareAbs();
}

float RobotPoseValidator::getSqrDistanceToLine(const Vector2<>& base, const Vector2<>& dir, const Vector2<>& point) const
{
  float l = (point.x - base.x) * dir.x + (point.y - base.y) * dir.y;
  return ((base + dir * l) - point).squareAbs();
}

Vector2<> RobotPoseValidator::getOrthogonalProjection(const Vector2<>& base, const Vector2<>& dir, const Vector2<>& point) const
{
  float l = (point.x - base.x) * dir.x + (point.y - base.y) * dir.y;
  return base + dir * l;
}

void RobotPoseValidator::motionUpdate()
{
  // add process noise
  translationCov[0][0] += sqr(p.filterProcessDeviation.translation.x);
  translationCov[1][1] += sqr(p.filterProcessDeviation.translation.y);
  rotationCov += sqr(p.filterProcessDeviation.rotation);

  // add odometry
  Pose2D odometryOffset = theOdometryData - lastOdometryData;
  Vector2<> odo(odometryOffset.translation.x, odometryOffset.translation.y);
  odo.rotate(rotation);
  translation += Vector<2>(odo.x, odo.y);
  rotation = normalize(rotation + odometryOffset.rotation);
  lastOdometryData = theOdometryData;

  // add odometry noise
  translationCov[0][0] += sqr(odo.x * p.odometryDeviation.translation.x);
  translationCov[1][1] += sqr(odo.y * p.odometryDeviation.translation.y);
  rotationCov += sqr(odometryOffset.rotation * p.odometryDeviation.rotation);
  rotationCov += sqr(odo.x * p.odometryRotationDeviation.x);
  rotationCov += sqr(odo.y * p.odometryRotationDeviation.y);
}

void RobotPoseValidator::sensorUpdate(float angle, float x, float y, float angleVariance, float xOrYVariance)
{
  if(angle < 100000.f)
  {
    COMPLEX_DRAWING("module:RobotPoseValidator:field",
    {
      Vector2<> dir(1000.f, 0.f);
      dir.rotate(angle);
      ARROW("module:RobotPoseValidator:field", translation.x, translation.y, translation.x + dir.x, translation.y + dir.y, 0, Drawings::ps_solid, ColorRGBA(0xff, 0, 0));
    });

    float k = rotationCov / (rotationCov + angleVariance);
    rotation = normalize(rotation + k * normalize(angle - rotation));
    rotationCov -= k * rotationCov;
  }

  ASSERT((x < 100000.f && y >= 100000.f) || (y < 100000.f && x >= 100000.f) || (x >= 100000.f && y >= 100000.f));
  if(x < 100000.f)
  {
    COMPLEX_DRAWING("module:RobotPoseValidator:field",
    {
      Vector2<> trans(x, translation.y);
      Vector2<> off(0, 1000.f);
      LINE("module:RobotPoseValidator:field", trans.x - off.x, trans.y - off.y, trans.x + off.x, trans.y + off.y, 0, Drawings::ps_solid, ColorRGBA(0xff, 0, 0));
    });

    Matrix<1, 1> translationSensorCov;
    translationSensorCov[0][0] = xOrYVariance;
    Vector<1> z;
    z[0] = x;
    Matrix<1, 2> c;
    c[0][0] = 1.f;
    Matrix<2, 1> cTrans = c.transpose();
    Matrix<2, 1> k = translationCov * cTrans * (c * translationCov * cTrans + translationSensorCov).invert();
    translation += k * (z - c * translation);
    translationCov -= k * c * translationCov;
  }
  if(y < 100000.f)
  {
    COMPLEX_DRAWING("module:RobotPoseValidator:field",
    {
      Vector2<> trans(translation.x, y);
      Vector2<> off(1000.f, 0.f);
      LINE("module:RobotPoseValidator:field", trans.x - off.x, trans.y - off.y, trans.x + off.x, trans.y + off.y, 0, Drawings::ps_solid, ColorRGBA(0xff, 0, 0));
    });

    Matrix<1, 1> translationSensorCov;
    translationSensorCov[0][0] = xOrYVariance;
    Vector<1> z;
    z[0] = y;
    Matrix<1, 2> c;
    c[1][0] = 1.f;
    Matrix<2, 1> cTrans = c.transpose();
    Matrix<2, 1> k = translationCov * cTrans * (c * translationCov * cTrans + translationSensorCov).invert();
    translation += k * (z - c * translation);
    translationCov -= k * c * translationCov;
  }
}

void RobotPoseValidator::useLinePercept(const LinePercept& linePercept, const FieldLine& fieldLine)
{
  Vector2<> center = (linePercept.start + linePercept.end) * 0.5f;
  Vector2<> dir = linePercept.end - linePercept.start;
  dir.normalize();
  Matrix2x2<> cov = getCovOfPointInWorld(center, 0.f);

  COMPLEX_DRAWING("module:RobotPoseValidator:field",
  {
    Vector2<> centerOnField = thePotentialRobotPose* center;
    float dist = (centerOnField.x - fieldLine.start.x) * fieldLine.dir.x + (centerOnField.y - fieldLine.start.y) * fieldLine.dir.y;
    Vector2<> foot = fieldLine.start + fieldLine.dir* dist;
    LINE("module:RobotPoseValidator:field", centerOnField.x, centerOnField.y, foot.x, foot.y, 0, Drawings::ps_solid, ColorRGBA(0, 0, 0xff));
  });
  COVARIANCE2D("module:RobotPoseValidator:fieldRel", cov, center);

  Vector2<> orthogonalProjection = getOrthogonalProjection(linePercept.start, dir, Vector2<>());
  float measuredAngle = -atan2(orthogonalProjection.y, orthogonalProjection.x);
  measuredAngle = normalize(measuredAngle + (fieldLine.vertical ? pi_2 : 0));
  float possibleAngle2 = normalize(measuredAngle - pi);
  if(abs(normalize(possibleAngle2 - thePotentialRobotPose.rotation)) < abs(normalize(measuredAngle - thePotentialRobotPose.rotation)))
    measuredAngle = possibleAngle2;
  float c = cos(measuredAngle), s = sin(measuredAngle);
  Matrix2x2<> angleRotationMatrix(Vector2<>(c, s), Vector2<>(-s, c));
  orthogonalProjection = angleRotationMatrix * orthogonalProjection;

  cov = angleRotationMatrix * cov * angleRotationMatrix.transpose();

  if(fieldLine.vertical)
  {
    float measuredY = fieldLine.start.y - orthogonalProjection.y;
    float yVariance = cov[1][1];
    float angleVariance = sqr(atan(sqrt(4.f * yVariance / (linePercept.start - linePercept.end).squareAbs())));
    sensorUpdate(measuredAngle, 100000.f, measuredY, angleVariance, yVariance);
  }
  else
  {
    float measuredX = fieldLine.start.x - orthogonalProjection.x;
    float xVariance = cov[0][0];
    float angleVariance = sqr(atan(sqrt(4.f * xVariance / (linePercept.start - linePercept.end).squareAbs())));
    sensorUpdate(measuredAngle, measuredX, 100000.f, angleVariance, xVariance);
  }
}

void RobotPoseValidator::useGoalPost(const Vector2<>& postPos, const Vector2<>& postOnField)
{
  float seenAngleToGoal = atan2(postPos.y, postPos.x);
  float measuredAngle = normalize((postOnField - Vector2<>(translation.x, translation.y)).angle() - seenAngleToGoal);

  Matrix2x2<> cov = getCovOfPointInWorld(postPos, 0.f);
  float c = cos(-seenAngleToGoal), s = sin(-seenAngleToGoal);
  Matrix2x2<> angleRotationMatrix(Vector2<>(c, s), Vector2<>(-s, c));
  cov = angleRotationMatrix * cov * angleRotationMatrix.transpose();
  float angleVariance = sqr(atan(sqrt(cov[1][1] / postPos.squareAbs())));

  sensorUpdate(measuredAngle, 100000.f, 100000.f, angleVariance, 100000.f);
}

void RobotPoseValidator::useCenterCircle(const Vector2<>& circlePos)
{
  Vector2<> measurement = -Vector2<>(circlePos).rotate(rotation);
  float circleDistance = circlePos.abs();
  Vector2<> increasedCirclePos = circlePos * ((circleDistance + theFieldDimensions.centerCircleRadius) / circleDistance);
  Matrix2x2<> cov = getCovOfPointInWorld(increasedCirclePos, 0.f);
  sensorUpdate(100000.f, 100000.f, measurement.y, 100000.f, cov[1][1]);
  sensorUpdate(100000.f, measurement.x, 100000.f, 100000.f, cov[0][0]);
}

Matrix2x2<> RobotPoseValidator::getCovOfPointInWorld(const Vector2<>& pointInWorld2, float pointZInWorld) const
{
  Vector3<> unscaledVectorToPoint = theCameraMatrix.invert() * Vector3<>(pointInWorld2.x, pointInWorld2.y, pointZInWorld);
  const Vector3<> unscaledWorld = theCameraMatrix.rotation * unscaledVectorToPoint;
  const float h = theCameraMatrix.translation.z - pointZInWorld;
  const float scale = h / -unscaledWorld.z;
  Vector2<> pointInWorld(unscaledWorld.x * scale, unscaledWorld.y * scale);
  const float distance = pointInWorld.abs();
  const float angle = pointInWorld.angle();
  const float c = cos(angle), s = sin(angle);
  Matrix2x2<> rot(Vector2<>(c, s), Vector2<>(-s, c));
  Matrix2x2<> cov(Vector2<>(sqr(h / tan((distance == 0.f ? pi_2 : atan(h / distance)) - p.robotRotationDeviation.x) - distance), 0.f),
                  Vector2<>(0.f, sqr(tan(p.robotRotationDeviation.y) * distance)));
  return rot * cov * rot.transpose();
}
