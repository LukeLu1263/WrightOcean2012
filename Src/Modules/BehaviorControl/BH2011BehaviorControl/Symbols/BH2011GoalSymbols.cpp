/**
 * @file BH2011GoalSymbols.cpp
 *
 * Implementation of class BH2011GoalSymbols.
 *
 * @author Tim Laue
 * @author Carsten Koenemann
 */

#include "BH2011GoalSymbols.h"

void BH2011GoalSymbols::registerSymbols(xabsl::Engine& engine)
{
  /*
   * free part angles
   */
  // angle to center of largest free part of opponent goal
  engine.registerDecimalInputSymbol("opponent_goal.free_part.angle_to_center", this, &BH2011GoalSymbols::getCenterAngleOfFreePart);
  // angle to outer side of largest free part of opponent goal
  engine.registerDecimalInputSymbol("opponent_goal.free_part.angle_to_outer_side", this, &BH2011GoalSymbols::getOuterAngleOfFreePart);
  // angle to inner side of largest free part of opponent goal
  engine.registerDecimalInputSymbol("opponent_goal.free_part.angle_to_inner_side", this, &BH2011GoalSymbols::getInnerAngleOfFreePart);
  // angle width of largest free part of opponent goal
  engine.registerDecimalInputSymbol("opponent_goal.free_part.angle_width", this, &BH2011GoalSymbols::getOpeningAngleOfFreePart);
  // angle tolerance to largest free part of opponent goal
  // (how accurate will the robot be positioned)
  engine.registerDecimalInputSymbol("opponent_goal.free_part.angle_tolerance", this, &BH2011GoalSymbols::getAngleToleranceToFreePart);
  /*
   * other perceptions
   */
  // time since opponent goal was last seen in ms
  engine.registerDecimalInputSymbol("opponent_goal.time_since_last_seen", &timeSinceOppGoalWasSeen);
  // was the opponent goal recently seen?
  engine.registerBooleanInputSymbol("opponent_goal.was_seen", &oppGoalWasSeen);
  // time since any goal was last seen in ms
  engine.registerDecimalInputSymbol("goal.time_since_last_seen_any_goal", &timeSinceAnyGoalWasSeen);
  engine.registerDecimalInputSymbol("goal.time_since_last_seen", &timeSinceAnyGoalWasSeen);
  // was any goal recently seen?
  engine.registerBooleanInputSymbol("goal.was_seen", &goalWasSeen);
  // count the number of seen goal posts in current ready state
  engine.registerDecimalInputSymbol("goal.number_of_sightings_in_ready", &goalsSeenInReadyState);
  // count the number of seen goal posts since last pickup
  engine.registerDecimalInputSymbol("goal.number_of_sightings_since_last_pickup", &goalsSeenSinceLastPickup);

  engine.registerDecimalInputSymbol("goal.angle_to_last_seen", this, &BH2011GoalSymbols::getAngleToLastSeen);
  engine.registerDecimalInputSymbol("goal.distance_to_last_seen", this, &BH2011GoalSymbols::getDistanceToLastSeen);

}

void BH2011GoalSymbols::update()
{
  timeSinceOppGoalWasSeen = (float) frameInfo.getTimeSince(goalPercept.timeWhenOppGoalLastSeen);
  timeSinceOwnGoalWasSeen = (float) frameInfo.getTimeSince(goalPercept.timeWhenOwnGoalLastSeen);
  timeSinceAnyGoalWasSeen = timeSinceOppGoalWasSeen < timeSinceOwnGoalWasSeen ? timeSinceOppGoalWasSeen : timeSinceOwnGoalWasSeen;
  oppGoalWasSeen          = timeSinceOppGoalWasSeen < 500;
  goalWasSeen             = timeSinceAnyGoalWasSeen < 500;

  /* Calculate seen goals in ready state*/
  if(gameInfo.state == STATE_READY)
  {
    if(frameInfo.getTimeSince(goalPercept.timeWhenOppGoalLastSeen) == 0 ||
       frameInfo.getTimeSince(goalPercept.timeWhenOwnGoalLastSeen) == 0)
      goalsSeenInReadyState += 1.0;
  }
  else
  {
    goalsSeenInReadyState = 0;
  }

  /* Calculate seen goals since last pickup */
  if((groundContactState.noContactSafe && damageConfiguration.useGroundContactDetectionForSafeStates)
     && (fallDownState.state == FallDownState::upright || fallDownState.state == FallDownState::staggering))
  {
    goalsSeenSinceLastPickup = 0.0;
  }
  else
  {
    if(frameInfo.getTimeSince(goalPercept.timeWhenOppGoalLastSeen) == 0 ||
       frameInfo.getTimeSince(goalPercept.timeWhenOwnGoalLastSeen) == 0)
      goalsSeenSinceLastPickup += 1.0;
  }
  PLOT("Behavior:goalsSeenSinceLastPickup", goalsSeenSinceLastPickup);

  /* determine angles to sides of free part of opponent goal */
  Vector2<> centerOfOppGoalAbs = Vector2<>((float) fieldDimensions.xPosOpponentGoalpost, (float)(fieldDimensions.yPosLeftGoal + fieldDimensions.yPosRightGoal) / 2.0f);
  Vector2<> centerOfOppGoalRel = Geometry::fieldCoord2Relative(robotPose, centerOfOppGoalAbs);
  float     leftDistance       = abs(centerOfOppGoalAbs.y - freePartOfOpponentGoalModel.leftEnd.y);
  float     rightDistance      = abs(centerOfOppGoalAbs.y - freePartOfOpponentGoalModel.rightEnd.y);
  if(leftDistance < rightDistance)
  {
    innerSide = freePartOfOpponentGoalModel.leftEnd;
    outerSide = freePartOfOpponentGoalModel.rightEnd;
  }
  else
  {
    innerSide = freePartOfOpponentGoalModel.rightEnd;
    outerSide = freePartOfOpponentGoalModel.leftEnd;
  }

  /* draw angles to free part into worldstate, see draw()-function below */
  DECLARE_DEBUG_DRAWING("behavior:GoalSymbols:FreePartAnglesOnField", "drawingOnField");
  COMPLEX_DRAWING("behavior:GoalSymbols:FreePartAnglesOnField", drawingOnField());
}

void BH2011GoalSymbols::drawingOnField()
{
  float dist = Geometry::distance(robotPose.translation, Vector2<>((float) fieldDimensions.xPosOpponentGroundline, (float)(fieldDimensions.yPosLeftGoal + fieldDimensions.yPosRightGoal) / 2.0f));
  Vector2<> targetImage;
  // draw angle to center of free part
  targetImage = Geometry::relative2FieldCoord(robotPose, dist * cos(fromDegrees(getCenterAngleOfFreePart())), dist * sin(fromDegrees(getCenterAngleOfFreePart())));
  LINE(
    "behavior:GoalSymbols:FreePartAnglesOnField",
    robotPose.translation.x, robotPose.translation.y, targetImage.x, targetImage.y,
    15, Drawings::ps_solid, ColorClasses::black
  );
  // draw angle to inner side of free part
  targetImage = Geometry::relative2FieldCoord(robotPose, dist * cos(fromDegrees(getInnerAngleOfFreePart())), dist * sin(fromDegrees(getInnerAngleOfFreePart())));
  LINE(
    "behavior:GoalSymbols:FreePartAnglesOnField",
    robotPose.translation.x, robotPose.translation.y, targetImage.x, targetImage.y,
    15, Drawings::ps_solid, ColorClasses::red
  );
  // draw angle to outer side of free part
  targetImage = Geometry::relative2FieldCoord(robotPose, dist * cos(fromDegrees(getOuterAngleOfFreePart())), dist * sin(fromDegrees(getOuterAngleOfFreePart())));
  LINE(
    "behavior:GoalSymbols:FreePartAnglesOnField",
    robotPose.translation.x, robotPose.translation.y, targetImage.x, targetImage.y,
    15, Drawings::ps_solid, ColorClasses::blue
  );
}

/*
 * this stuff gets passed to xabsl
 */

float BH2011GoalSymbols::getCenterAngleOfFreePart()
{
  Pose2D    poseForOppGoalAngle = robotPose;
  Vector2<> leftOppGoalPostAbs  = Vector2<>((float) fieldDimensions.xPosOpponentGroundline, (float) fieldDimensions.yPosLeftGoal);
  Vector2<> rightOppGoalPostAbs = Vector2<>((float) fieldDimensions.xPosOpponentGroundline, (float) fieldDimensions.yPosRightGoal);
  if(poseForOppGoalAngle.translation.x > float(fieldDimensions.xPosOpponentGroundline - 50))
  {
    poseForOppGoalAngle.translation.x = float(fieldDimensions.xPosOpponentGroundline - 50);
  }
  float leftOppGoalPostAngle  = Geometry::angleTo(poseForOppGoalAngle, leftOppGoalPostAbs);
  float rightOppGoalPostAngle = Geometry::angleTo(poseForOppGoalAngle, rightOppGoalPostAbs);
  if(leftOppGoalPostAngle < rightOppGoalPostAngle)
  {
    leftOppGoalPostAngle += pi2;
  }
  Vector2<> centerOfFreePart = (freePartOfOpponentGoalModel.leftEnd + freePartOfOpponentGoalModel.rightEnd) / 2.0f;
  float angleToCenterOfFreePart = Range<>(rightOppGoalPostAngle - getAngleToleranceToFreePart(), leftOppGoalPostAngle + getAngleToleranceToFreePart()).limit(centerOfFreePart.angle());
  return toDegrees(normalize(angleToCenterOfFreePart));
}

float BH2011GoalSymbols::getOuterAngleOfFreePart()
{
  return toDegrees(normalize(outerSide.angle()));
}

float BH2011GoalSymbols::getInnerAngleOfFreePart()
{
  return toDegrees(normalize(innerSide.angle()));
}

float BH2011GoalSymbols::getAngleToleranceToFreePart()
{
  return max(10.0f, getOpeningAngleOfFreePart() - 10.0f);
}

float BH2011GoalSymbols::getOpeningAngleOfFreePart()
{
  float angleToLeftEnd  = freePartOfOpponentGoalModel.leftEnd.angle();
  float angleToRightEnd = freePartOfOpponentGoalModel.rightEnd.angle();
  if(angleToLeftEnd < angleToRightEnd)
  {
    angleToLeftEnd += pi2;
  }
  return toDegrees(abs(normalize(angleToLeftEnd - angleToRightEnd)));
}

float BH2011GoalSymbols::getAngleToLastSeen()
{
  Vector2<int> position;
  unsigned int maxLastSeen = 0;
  for(int i = 0; i < GoalPercept::NUMBER_OF_GOAL_POSTS; ++i)
    if(goalPercept.posts[i].timeWhenLastSeen > maxLastSeen)
    {
      maxLastSeen = goalPercept.posts[i].timeWhenLastSeen;
      position = goalPercept.posts[i].positionOnField;
    }
  for(int i = 0; i < GoalPercept::NUMBER_OF_UNKNOWN_GOAL_POSTS; ++i)
    if(goalPercept.posts[i].timeWhenLastSeen > maxLastSeen)
    {
      maxLastSeen = goalPercept.posts[i].timeWhenLastSeen;
      position = goalPercept.posts[i].positionOnField;
    }
  return toDegrees(Vector2<>(float(position.x), float(position.y)).angle());
}

float BH2011GoalSymbols::getDistanceToLastSeen()
{
  Vector2<int> position;
  unsigned int maxLastSeen = 0;
  for(int i = 0; i < GoalPercept::NUMBER_OF_GOAL_POSTS; ++i)
    if(goalPercept.posts[i].timeWhenLastSeen > maxLastSeen)
    {
      maxLastSeen = goalPercept.posts[i].timeWhenLastSeen;
      position = goalPercept.posts[i].positionOnField;
    }
  for(int i = 0; i < GoalPercept::NUMBER_OF_UNKNOWN_GOAL_POSTS; ++i)
    if(goalPercept.posts[i].timeWhenLastSeen > maxLastSeen)
    {
      maxLastSeen = goalPercept.posts[i].timeWhenLastSeen;
      position = goalPercept.posts[i].positionOnField;
    }
  return Vector2<>(float(position.x), float(position.y)).abs();
}
