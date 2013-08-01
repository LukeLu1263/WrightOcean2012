/** 
* \file BH2011SoccerSymbols.cpp
* Implementation of symbols for our 4 roles.
* \author Colin Graf
* \author <a href="mailto:aschreck@informatik.uni-bremen.de">Andr&eacute; Schreck</a>
* \author Katharina Gillmann
*/

#include <limits>
#include "Tools/Debugging/Modify.h"
#include "BH2011SoccerSymbols.h"
#include "BH2011TeamSymbols.h"
#include "BH2011LocatorSymbols.h"
#include "BH2011BallSymbols.h"
#include "Tools/Team.h"


void BH2011SoccerSymbols::registerSymbols(xabsl::Engine& engine)
{
  engine.registerEnumElement("soccer.behavior_action", "soccer.behavior_action.unknown", BehaviorData::unknown);
  engine.registerEnumElement("soccer.behavior_action", "soccer.behavior_action.dribble", BehaviorData::dribble);
  engine.registerEnumElement("soccer.behavior_action", "soccer.behavior_action.go_to_ball", BehaviorData::goToBall);
  engine.registerEnumElement("soccer.behavior_action", "soccer.behavior_action.search_for_ball", BehaviorData::searchForBall);
  engine.registerEnumElement("soccer.behavior_action", "soccer.behavior_action.go_to_target", BehaviorData::goToTarget);
  engine.registerEnumElement("soccer.behavior_action", "soccer.behavior_action.prepare_kick", BehaviorData::prepareKick);
  engine.registerEnumElement("soccer.behavior_action", "soccer.behavior_action.kick", BehaviorData::kick);
  engine.registerEnumElement("soccer.behavior_action", "soccer.behavior_action.kick_sidewards", BehaviorData::kickSidewards);
  engine.registerEnumElement("soccer.behavior_action", "soccer.behavior_action.pass", BehaviorData::pass);
  engine.registerEnumElement("soccer.behavior_action", "soccer.behavior_action.block", BehaviorData::block);
  engine.registerEnumElement("soccer.behavior_action", "soccer.behavior_action.hold", BehaviorData::hold);
  engine.registerEnumElement("soccer.behavior_action", "soccer.behavior_action.stand_up", BehaviorData::standUp);
  engine.registerEnumElement("soccer.behavior_action", "soccer.behavior_action.patrol", BehaviorData::patrol);
  engine.registerEnumElement("soccer.behavior_action", "soccer.behavior_action.pass_before_goal", BehaviorData::passBeforeGoal);
  engine.registerEnumElement("soccer.behavior_action", "soccer.behavior_action.kickoff", BehaviorData::kickoff);
  engine.registerEnumElement("soccer.behavior_action", "soccer.behavior_action.wait_for_pass", BehaviorData::waitForPass);
  engine.registerEnumElement("soccer.behavior_action", "soccer.behavior_action.prepare_pass", BehaviorData::preparePass);
  engine.registerEnumeratedOutputSymbol("soccer.behavior_action", "soccer.behavior_action", (int*)&behaviorData.action);

  engine.registerBooleanInputSymbol("soccer.disable_pre_initial", &disablePreInitialState);

  engine.registerDecimalInputSymbol("soccer.opponent_goal_angle",this, &BH2011SoccerSymbols::SoccerOpponentGoalAngle);

  engine.registerDecimalInputSymbol("soccer.position_next_to_ball.x", this,&BH2011SoccerSymbols::computePositionNextBallX);
  engine.registerDecimalInputSymbolBooleanParameter("soccer.position_next_to_ball.x", "soccer.position_next_to_ball.x.side", &sideLeft);

  engine.registerDecimalInputSymbol("soccer.position_next_to_ball.y",this,&BH2011SoccerSymbols::computePositionNextBallY);
  engine.registerDecimalInputSymbolBooleanParameter("soccer.position_next_to_ball.y", "soccer.position_next_to_ball.y.side", &sideLeft);

  engine.registerDecimalInputSymbol("soccer.position_next_to_ball.angle", this, &BH2011SoccerSymbols::computeAngleNextBall);
  engine.registerDecimalInputSymbolBooleanParameter("soccer.position_next_to_ball.angle", "soccer.position_next_to_ball.angle.side", &sideLeft);

  engine.registerDecimalInputSymbol("soccer.position_behind_ball.x", this, &BH2011SoccerSymbols::computePositionBehindBallX);
  engine.registerDecimalInputSymbol("soccer.position_behind_ball.y", this, &BH2011SoccerSymbols::computePositionBehindBallY);
  engine.registerDecimalInputSymbol("soccer.position_behind_ball.angle", this, &BH2011SoccerSymbols::computePositionBehindBallAngle);

  /** The desired goalie's position on the field. */
  engine.registerDecimalInputSymbol("soccer.desired_keeper_pose.x", this, &BH2011SoccerSymbols::getDesiredKeeperPoseX);
  engine.registerDecimalInputSymbol("soccer.desired_keeper_pose.y", this, &BH2011SoccerSymbols::getDesiredKeeperPoseY);
  engine.registerDecimalInputSymbol("soccer.desired_keeper_pose.angle", this, &BH2011SoccerSymbols::getDesiredKeeperPoseAngle);
  engine.registerBooleanInputSymbol("soccer.desired_keeper_pose.reached", &desiredKeeperPoseReached);
}


void BH2011SoccerSymbols::init()
{
#ifdef TARGET_SIM
  disablePreInitialState = true;
#else
  disablePreInitialState = Global::getSettings().recover;
#endif
}

void BH2011SoccerSymbols::update()
{
  	DECLARE_DEBUG_DRAWING("DesiredkeeperPose", "drawingOnField");
	COMPLEX_DRAWING("DesiredkeeperPose",
  {
	LINE("DesiredkeeperPose", ball.x, ball.y, M.x, M.y, 5, Drawings::ps_solid, ColorClasses::black);
	LINE("DesiredkeeperPose", ball.x, ball.y, goalPostLeft.x, goalPostLeft.y, 5, Drawings::ps_solid, ColorClasses::white);
	LINE("DesiredkeeperPose", ball.x, ball.y, goalPostRight.x, goalPostRight.y, 5, Drawings::ps_solid, ColorClasses::white);
  });
	isDesiredKeeperPoseReached();
}

float BH2011SoccerSymbols::soccerOpponentGoalAngle()
{
  if(frameInfo.getTimeSince(goalPercept.timeWhenOppGoalLastSeen) < 4000) 
  {
    float angleLeft = toDegrees(goalPercept.posts[GoalPercept::LEFT_OPPONENT].positionOnField.angle());
    float angleRight = toDegrees(goalPercept.posts[GoalPercept::RIGHT_OPPONENT].positionOnField.angle());
    return (angleLeft + angleRight) / 2;
  }
  else
  {
    return toDegrees(Geometry::angleTo(robotPose, Vector2<float>((float)fieldDimensions.xPosOpponentGroundline, 0)));
  }
}

float BH2011SoccerSymbols::computePositionNextBallX()
{
  return computePosition().x;
}

float BH2011SoccerSymbols::computePositionNextBallY()
{
  return computePosition().y;
}

float BH2011SoccerSymbols::computeAngleNextBall()
{
  return toDegrees(Geometry::angleTo(robotPose,computePosition()));
}

Vector2<float> BH2011SoccerSymbols::computePosition()
{
  Vector2<float> ballPosition = ballModel.estimate.getPositionInFieldCoordinates(robotPose);
  Vector2<float> temp = ballPosition - Vector2<float>((float)fieldDimensions.xPosOpponentGroundline, 0);
  if(sideLeft) temp.rotateLeft();
  else temp.rotateRight();
  temp.normalize(300);
  return (temp + ballPosition);
}

float BH2011SoccerSymbols::computePositionBehindBallX()
{
  return computePositionBehindBall().x;
}

float BH2011SoccerSymbols::computePositionBehindBallY()
{
  return computePositionBehindBall().y;
}

float BH2011SoccerSymbols::computePositionBehindBallAngle()
{
  return toDegrees(Geometry::angleTo(robotPose,computePositionBehindBall()));
}

Vector2<float> BH2011SoccerSymbols::computePositionBehindBall()
{
  Vector2<float> ballPosition = ballModel.estimate.getPositionInFieldCoordinates(robotPose);
  Vector2<float> temp = ballPosition - Vector2<float>((float)fieldDimensions.xPosOpponentGroundline, 0);
  temp.normalize(250);
  return (temp + ballPosition);
}

//absolute x-position
float BH2011SoccerSymbols::getDesiredKeeperPoseX()
{
	clipMinX = -2720;
	clipMaxX = -2700;
	if(robotPose.translation.x < clipMinX) return clipMinX;
    else if(robotPose.translation.x >= clipMaxX) return clipMaxX;
	else return robotPose.translation.x;
}

//absolute y-position
float BH2011SoccerSymbols::getDesiredKeeperPoseY()
{
	//计算keeper在守门时应当站在角平分线上的y值.
	//According to ball position on field, compute the position y that keeper should reach the y. 
	//The y is the intersection of the angular bisector between the left goalpost and right goalpost to ball posotion.
	
	//Now, we know the vertexes of ABC triangle
	Vector2<float> A(-3000, 700);
	Vector2<float> B(-3000, -700);

	//vertex A(x1,y1)
	goalPostLeft = A;
	//vertex B(x2,y2)
	ball = ballModel.estimate.getPositionInFieldCoordinates(robotPose); //ball.on_field
	//vertex C(x3,y3)
	goalPostRight = B;
	
	// a = |BC|
	float a = (ball - goalPostRight).abs(); 
	// b = |AC|
	float b = (goalPostLeft - goalPostRight).abs();
	// c = |AB|
	float c = (goalPostLeft - ball).abs(); 

	/*according to equality of incenter of triangle, compute the incenter point M.
	  equality of incenter of triangle:
	  ( (a*x1 + b*x2 + c*x3)/(a + b + c), (a*y1 + b*y2 + c*y3)/(a + b + c) )
	  */
	M.x = (a*goalPostLeft.x + b*ball.x + c*goalPostRight.x)/(a + b + c);
	M.y = (a*goalPostLeft.y + b*ball.y + c*goalPostRight.y)/(a + b + c);

	//keeper所在位置的x方向值. 该直线为平行于endLine的直线，与角平分线相交处的y值即为keeperPoseY的值.
	float x = robotPose.translation.x; //locator.pose.x
	float k = (ball.y - M.y)/(ball.x - M.x);
	float y = k*(x - M.x) + M.y;
	
	clipMinY = -750;
	clipMaxY = 750;
	if(y < clipMinY) return clipMinY;
    else if(y > clipMaxY) return clipMaxY;
	else return y;
}

//relative angular
float BH2011SoccerSymbols::getDesiredKeeperPoseAngle()
{
	clipMinAngle = -70;
	clipMaxAngle = 70;
	if(toDegrees(robotPose.getAngle()) < clipMinAngle)
		return -toDegrees(robotPose.getAngle()) + clipMinAngle;
    else if(toDegrees(robotPose.getAngle()) > clipMaxAngle) 
		return -toDegrees(robotPose.getAngle()) + clipMaxAngle;
	else
		return toDegrees(ballModel.estimate.getAngle());//ball.angle
}


void BH2011SoccerSymbols::isDesiredKeeperPoseReached()
{
	if(robotPose.translation.x > getDesiredKeeperPoseX()-50 && robotPose.translation.x < getDesiredKeeperPoseX()+50
	&& robotPose.translation.y > getDesiredKeeperPoseY()-50 && robotPose.translation.y < getDesiredKeeperPoseY()+50
	&& abs(getDesiredKeeperPoseAngle()) < 5)
		desiredKeeperPoseReached = true;
	else desiredKeeperPoseReached = false;
}