/**
* @file SampleTemplateGenerator.cpp
*
* This file implements a submodule that generates robot positions from percepts.
*
* @author <a href="mailto:Tim.Laue@dfki.de">Tim Laue</a>
*/

#include "SampleTemplateGenerator.h"
#include "Tools/Math/Probabilistics.h"


SampleTemplateGenerator::SampleTemplateGenerator(const GoalPercept& goalPercept, const LinePercept& linePercept,
                                                 const FrameInfo& frameInfo,
                                                 const FieldDimensions& fieldDimensions, 
                                                 const OdometryData& odometryData,
                                                 const float& standardDeviationGoalpostSampleBearingDistance,
                                                 const float& standardDeviationGoalpostSampleSizeDistance,
                                                 const bool& clipTemplateGeneration,
                                                 const Range<>& clipTemplateGenerationRangeX,
                                                 const Range<>& clipTemplateGenerationRangeY,
                                                 const int& templateKeepMaxTime):
  theGoalPercept(goalPercept),
  theLinePercept(linePercept),
  theFrameInfo(frameInfo),
  theFieldDimensions(fieldDimensions),
  theOdometryData(odometryData),
  standardDeviationGoalpostSampleBearingDistance(standardDeviationGoalpostSampleBearingDistance),
  standardDeviationGoalpostSampleSizeDistance(standardDeviationGoalpostSampleSizeDistance),
  clipTemplateGeneration(clipTemplateGeneration),
  clipTemplateGenerationRangeX(clipTemplateGenerationRangeX),
  clipTemplateGenerationRangeY(clipTemplateGenerationRangeY),
  templateKeepMaxTime(templateKeepMaxTime)
{
}

void SampleTemplateGenerator::init()
{
  realPostPositions[GoalPercept::LEFT_OPPONENT] =
    Vector2<>((float) theFieldDimensions.xPosOpponentGoalpost, (float) theFieldDimensions.yPosLeftGoal);
  realPostPositions[GoalPercept::RIGHT_OPPONENT] =
    Vector2<>((float) theFieldDimensions.xPosOpponentGoalpost, (float) theFieldDimensions.yPosRightGoal);
  realPostPositions[GoalPercept::LEFT_OWN] =
    Vector2<>((float) theFieldDimensions.xPosOwnGoalpost, (float) theFieldDimensions.yPosRightGoal); //y coordinates are switched
  realPostPositions[GoalPercept::RIGHT_OWN]  =
    Vector2<>((float) theFieldDimensions.xPosOwnGoalpost, (float) theFieldDimensions.yPosLeftGoal);  //y coordinates are switched
}

void SampleTemplateGenerator::bufferNewPerceptions()
{
  // Buffer data generated from GoalPercept:
  // We currently see the complete opponent goal:
  if((theGoalPercept.posts[GoalPercept::LEFT_OPPONENT].timeWhenLastSeen == theFrameInfo.time) &&
     (theGoalPercept.posts[GoalPercept::RIGHT_OPPONENT].timeWhenLastSeen == theFrameInfo.time) &&
     (theGoalPercept.posts[GoalPercept::LEFT_OPPONENT].distanceType != GoalPost::IS_CLOSER) &&
     (theGoalPercept.posts[GoalPercept::RIGHT_OPPONENT].distanceType != GoalPost::IS_CLOSER))
  {
    FullGoal newFullGoal;
    newFullGoal.realLeftPosition  = realPostPositions[GoalPercept::LEFT_OPPONENT];
    newFullGoal.realRightPosition = realPostPositions[GoalPercept::RIGHT_OPPONENT];
    newFullGoal.seenLeftPosition.x = (float) theGoalPercept.posts[GoalPercept::LEFT_OPPONENT].positionOnField.x;
    newFullGoal.seenLeftPosition.y = (float) theGoalPercept.posts[GoalPercept::LEFT_OPPONENT].positionOnField.y;
    newFullGoal.seenRightPosition.x = (float) theGoalPercept.posts[GoalPercept::RIGHT_OPPONENT].positionOnField.x;
    newFullGoal.seenRightPosition.y = (float) theGoalPercept.posts[GoalPercept::RIGHT_OPPONENT].positionOnField.y;
    newFullGoal.timestamp = theFrameInfo.time;
    newFullGoal.odometry = theOdometryData;
    // Before adding, check if templates can be generated from this perception
    SampleTemplate checkTemplate = generateTemplateFromFullGoal(newFullGoal);
    if(checkTemplate.timestamp)
      fullGoals.add(newFullGoal);
  }
  // We currently see the complete own goal:
  else if((theGoalPercept.posts[GoalPercept::LEFT_OWN].timeWhenLastSeen == theFrameInfo.time) &&
          (theGoalPercept.posts[GoalPercept::RIGHT_OWN].timeWhenLastSeen == theFrameInfo.time) &&
          (theGoalPercept.posts[GoalPercept::LEFT_OWN].distanceType != GoalPost::IS_CLOSER) &&
          (theGoalPercept.posts[GoalPercept::RIGHT_OWN].distanceType != GoalPost::IS_CLOSER))
  {
    FullGoal newFullGoal;
    newFullGoal.realLeftPosition  = realPostPositions[GoalPercept::LEFT_OWN];
    newFullGoal.realRightPosition = realPostPositions[GoalPercept::RIGHT_OWN];
    newFullGoal.seenLeftPosition.x = (float) theGoalPercept.posts[GoalPercept::LEFT_OWN].positionOnField.x;
    newFullGoal.seenLeftPosition.y = (float) theGoalPercept.posts[GoalPercept::LEFT_OWN].positionOnField.y;
    newFullGoal.seenRightPosition.x = (float) theGoalPercept.posts[GoalPercept::RIGHT_OWN].positionOnField.x;
    newFullGoal.seenRightPosition.y = (float) theGoalPercept.posts[GoalPercept::RIGHT_OWN].positionOnField.y;
    newFullGoal.timestamp = theFrameInfo.time;
    newFullGoal.odometry = theOdometryData;
    // Before adding, check if templates can be generated from this perception
    SampleTemplate checkTemplate = generateTemplateFromFullGoal(newFullGoal);
    if(checkTemplate.timestamp)
      fullGoals.add(newFullGoal);
  }
  // We might currently see a single goal post with known side (but not a complete goal)
  else
  {
    for(int p = 0; p < GoalPercept::NUMBER_OF_GOAL_POSTS; p++)
    {
      const GoalPost& post = theGoalPercept.posts[p];
      if((post.timeWhenLastSeen == theFrameInfo.time) &&
         (post.distanceType != GoalPost::IS_CLOSER))
      {
        KnownGoalpost newPost;
        newPost.realPosition = realPostPositions[p];
        newPost.seenPosition.x = (float) post.positionOnField.x;
        newPost.seenPosition.y = (float) post.positionOnField.y;
        newPost.timestamp = theFrameInfo.time;
        newPost.odometry = theOdometryData;
        newPost.centerCircleSeen = theLinePercept.circle.found;
        if(newPost.centerCircleSeen)
          newPost.centerCircleSeenPosition = Vector2<>((float)(theLinePercept.circle.pos.x), (float)(theLinePercept.circle.pos.y));
        knownGoalposts.add(newPost);
      }
    }
  }
  // Maybe we have seen some goalpost of which we do not know the side:
  for(int p = 0; p < GoalPercept::NUMBER_OF_UNKNOWN_GOAL_POSTS; p++)
  {
    const GoalPost& post = theGoalPercept.unknownPosts[p];
    if((post.timeWhenLastSeen == theFrameInfo.time) &&
       (post.distanceType != GoalPost::IS_CLOSER))
    {
      UnknownGoalpost newPost;
      newPost.realPositions[0] = realPostPositions[2 * p];
      newPost.realPositions[1] = realPostPositions[2 * p + 1];
      newPost.seenPosition.x = (float) post.positionOnField.x;
      newPost.seenPosition.y = (float) post.positionOnField.y;
      newPost.timestamp = theFrameInfo.time;
      newPost.odometry = theOdometryData;
      newPost.centerCircleSeen = theLinePercept.circle.found;
      if(newPost.centerCircleSeen)
        newPost.centerCircleSeenPosition = Vector2<>((float)(theLinePercept.circle.pos.x), (float)(theLinePercept.circle.pos.y));
      unknownGoalposts.add(newPost);
    }
  }
  // If there are still some too old percepts after adding new ones -> delete them:
  removeOldPercepts(fullGoals);
  removeOldPercepts(knownGoalposts);
  removeOldPercepts(unknownGoalposts);
}

template<typename T>
void SampleTemplateGenerator::removeOldPercepts(RingBuffer<T, MAX_PERCEPTS>& buffer)
{
  while(buffer.getNumberOfEntries())
  {
    T& oldestElement = buffer[buffer.getNumberOfEntries() - 1];
    if(theFrameInfo.getTimeSince(oldestElement.timestamp) > templateKeepMaxTime)
      buffer.removeFirst();
    else
      break;
  }
}

SampleTemplate SampleTemplateGenerator::getNewTemplate()
{
  SampleTemplate newTemplate;
  // Current solution: Prefer to construct templates from full goals only:
  if(fullGoals.getNumberOfEntries())
  {
    FullGoal& goal = fullGoals[rand() % fullGoals.getNumberOfEntries()];
    newTemplate = generateTemplateFromFullGoal(goal);
  }
  else if(knownGoalposts.getNumberOfEntries())
  {
    KnownGoalpost& goalPost = knownGoalposts[rand() % knownGoalposts.getNumberOfEntries()];
    if(goalPost.centerCircleSeen)
      newTemplate = generateTemplateFromPositionAndCenterCircle(goalPost.seenPosition, goalPost.centerCircleSeenPosition, goalPost.realPosition, goalPost.odometry);
    if(newTemplate.timestamp == 0)
      newTemplate = generateTemplateFromPosition(goalPost.seenPosition, goalPost.realPosition, goalPost.odometry);
  }
  else if(unknownGoalposts.getNumberOfEntries())
  {
    UnknownGoalpost& goalPost = unknownGoalposts[rand() % unknownGoalposts.getNumberOfEntries()];
    if(goalPost.centerCircleSeen)
      newTemplate = generateTemplateFromPositionAndCenterCircle(goalPost.seenPosition, goalPost.centerCircleSeenPosition, goalPost.realPositions[rand() % 2], goalPost.odometry);
    if(newTemplate.timestamp == 0)
      newTemplate = generateTemplateFromPosition(goalPost.seenPosition, goalPost.realPositions[rand() % 2], goalPost.odometry);
  }
  if(newTemplate.timestamp == 0) // In some cases, no proper sample is generated, return a random sample
  {
    newTemplate = generateRandomTemplate();
  }
  return newTemplate;
}

bool SampleTemplateGenerator::templatesAvailable() const
{
  const int sumOfTemplates = fullGoals.getNumberOfEntries() +
                             knownGoalposts.getNumberOfEntries() + unknownGoalposts.getNumberOfEntries();
  return sumOfTemplates > 0;
}

SampleTemplate SampleTemplateGenerator::generateTemplateFromFullGoal(const FullGoal& goal) const
{
  SampleTemplate newTemplate;
  Pose2D odometryOffset = theOdometryData - goal.odometry;
  float leftPostDist = goal.seenLeftPosition.abs();
  float leftDistUncertainty = sampleTriangularDistribution(standardDeviationGoalpostSampleBearingDistance);
  if(leftPostDist + leftDistUncertainty > standardDeviationGoalpostSampleBearingDistance)
    leftPostDist += leftDistUncertainty;
  float rightPostDist = goal.seenRightPosition.abs();
  float rightDistUncertainty = sampleTriangularDistribution(standardDeviationGoalpostSampleBearingDistance);
  if(rightPostDist + rightDistUncertainty > standardDeviationGoalpostSampleBearingDistance)
    rightPostDist += rightDistUncertainty;
  Geometry::Circle c1(goal.realLeftPosition, leftPostDist + theFieldDimensions.goalPostRadius);
  Geometry::Circle c2(goal.realRightPosition, rightPostDist + theFieldDimensions.goalPostRadius);
  // If there are intersections, take the first one that is in the field:
  Vector2<> p1, p2;
  int result = Geometry::getIntersectionOfCircles(c1, c2, p1, p2);
  if(result)
  {
    if(theFieldDimensions.isInsideCarpet(p1) && checkTemplateClipping(p1))
    {
      float origAngle = (goal.realLeftPosition - p1).angle();
      float observedAngle = goal.seenLeftPosition.angle();
      Pose2D templatePose(origAngle - observedAngle, p1);
      templatePose += odometryOffset;
      newTemplate = templatePose;
      newTemplate.timestamp = theFrameInfo.time;
    }
    else if(theFieldDimensions.isInsideCarpet(p2) && checkTemplateClipping(p2))
    {
      float origAngle = (goal.realLeftPosition - p2).angle();
      float observedAngle = goal.seenLeftPosition.angle();
      Pose2D templatePose(origAngle - observedAngle, p2);
      templatePose += odometryOffset;
      newTemplate = templatePose;
      newTemplate.timestamp = theFrameInfo.time;
    }
  }
  // The else case is omitted, calling function has to check the timestamp of the generated sample
  return newTemplate;
}

SampleTemplate SampleTemplateGenerator::generateTemplateFromPositionAndCenterCircle(const Vector2<>& posSeen, const Vector2<>& circlePosSeen,
    const Vector2<>& posReal, const Pose2D& postOdometry) const
{
  SampleTemplate newTemplate;
  Pose2D odometryOffset = theOdometryData - postOdometry;
  float postDist = posSeen.abs();
  float postDistUncertainty = sampleTriangularDistribution(standardDeviationGoalpostSampleBearingDistance);
  if(postDist + postDistUncertainty > standardDeviationGoalpostSampleBearingDistance)
    postDist += postDistUncertainty;
  float circleDist = circlePosSeen.abs();
  float circleDistUncertainty = sampleTriangularDistribution(standardDeviationGoalpostSampleBearingDistance); //No special uncertainty for center circle available
  if(circleDist + circleDistUncertainty > standardDeviationGoalpostSampleBearingDistance)
    circleDist += circleDistUncertainty;
  Geometry::Circle c1(posReal, postDist + theFieldDimensions.goalPostRadius);
  Geometry::Circle c2(Vector2<>(0.0f, 0.0f), circleDist);
  // If there are intersections, take the first one that is in the field:
  Vector2<> p1, p2;
  int result = Geometry::getIntersectionOfCircles(c1, c2, p1, p2);
  if(result)
  {
    if(theFieldDimensions.isInsideCarpet(p1) && checkTemplateClipping(p1))
    {
      float origAngle = (posReal - p1).angle();
      float observedAngle = posSeen.angle();
      Pose2D templatePose(origAngle - observedAngle, p1);
      templatePose += odometryOffset;
      newTemplate = templatePose;
      newTemplate.timestamp = theFrameInfo.time;
    }
    else if(theFieldDimensions.isInsideCarpet(p2) && checkTemplateClipping(p2))
    {
      float origAngle = (posReal - p2).angle();
      float observedAngle = posSeen.angle();
      Pose2D templatePose(origAngle - observedAngle, p2);
      templatePose += odometryOffset;
      newTemplate = templatePose;
      newTemplate.timestamp = theFrameInfo.time;
    }
  }
  // The else case is omitted, calling function has to check the timestamp of the generated sample
  return newTemplate;
}

SampleTemplate SampleTemplateGenerator::generateTemplateFromPosition(
  const Vector2<>& posSeen, const Vector2<>& posReal,
  const Pose2D& postOdometry) const
{
  SampleTemplate newTemplate;
  float r = posSeen.abs() + theFieldDimensions.goalPostRadius;
  float distUncertainty = sampleTriangularDistribution(standardDeviationGoalpostSampleBearingDistance);
  if(r + distUncertainty > standardDeviationGoalpostSampleBearingDistance)
    r += distUncertainty;
  Vector2<> realPosition = posReal;
  float minY = std::max(posReal.y - r, static_cast<float>(theFieldDimensions.yPosRightFieldBorder));
  float maxY = std::min(posReal.y + r, static_cast<float>(theFieldDimensions.yPosLeftFieldBorder));
  Vector2<> p;
  p.y = minY + randomFloat() * (maxY - minY);
  float xOffset(sqrt(sqr(r) - sqr(p.y - posReal.y)));
  p.x = posReal.x;
  p.x += (p.x > 0) ? -xOffset : xOffset;
  if(theFieldDimensions.isInsideCarpet(p) && checkTemplateClipping(p))
  {
    float origAngle = (realPosition - p).angle();
    float observedAngle = posSeen.angle();
    Pose2D templatePose(origAngle - observedAngle, p);
    Pose2D odometryOffset = theOdometryData - postOdometry;
    templatePose += odometryOffset;
    newTemplate = templatePose;
    newTemplate.timestamp = theFrameInfo.time;
  }
  return newTemplate;
}

SampleTemplate SampleTemplateGenerator::generateRandomTemplate() const
{
  SampleTemplate newTemplate;
  if(clipTemplateGeneration)
    newTemplate = Pose2D::random(clipTemplateGenerationRangeX, clipTemplateGenerationRangeY, Range<>(-pi, pi));
  else
    newTemplate = theFieldDimensions.randomPoseOnField();
  newTemplate.timestamp = theFrameInfo.time;
  return newTemplate;
}

void SampleTemplateGenerator::draw()
{
  for(int i = 0; i < fullGoals.getNumberOfEntries(); ++i)
  {
    FullGoal& goal = fullGoals[i];
    Pose2D odometryOffset = goal.odometry - theOdometryData;
    Vector2<> leftPost = odometryOffset * goal.seenLeftPosition;
    Vector2<> rightPost = odometryOffset * goal.seenRightPosition;
    LINE("module:SelfLocator:templates", leftPost.x, leftPost.y,
         rightPost.x, rightPost.y, 50, Drawings::ps_solid, ColorRGBA(140, 140, 255));
    CIRCLE("module:SelfLocator:templates", leftPost.x, leftPost.y,
           100, 20, Drawings::ps_solid, ColorRGBA(0, 0, 0), Drawings::bs_solid, ColorRGBA(140, 140, 255));
    CIRCLE("module:SelfLocator:templates", rightPost.x, rightPost.y,
           100, 20, Drawings::ps_solid, ColorRGBA(0, 0, 0), Drawings::bs_solid, ColorRGBA(140, 140, 255));
  }
  for(int i = 0; i < knownGoalposts.getNumberOfEntries(); ++i)
  {
    KnownGoalpost& post = knownGoalposts[i];
    Pose2D odometryOffset = post.odometry - theOdometryData;
    Vector2<> postPos = odometryOffset * post.seenPosition;
    CIRCLE("module:SelfLocator:templates", postPos.x, postPos.y,
           100, 20, Drawings::ps_solid, ColorRGBA(140, 140, 255), Drawings::bs_solid, ColorRGBA(140, 140, 255));
    CIRCLE("module:SelfLocator:templates", postPos.x, postPos.y,
           200, 20, Drawings::ps_solid, ColorRGBA(0, 0, 0), Drawings::bs_null, ColorRGBA(140, 140, 255));
  }
}

bool SampleTemplateGenerator::checkTemplateClipping(const Vector2<> pos) const
{
  if(!clipTemplateGeneration)
    return true;
  else
    return (clipTemplateGenerationRangeX.isInside(pos.x) && clipTemplateGenerationRangeY.isInside(pos.y));
}
