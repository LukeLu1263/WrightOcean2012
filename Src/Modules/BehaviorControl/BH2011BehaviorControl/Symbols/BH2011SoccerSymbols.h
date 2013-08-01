/** 
* \file BH2011SoccerSymbols.h
* Implementation of symbols for our 4 roles.
* \author Colin Graf
*/

#ifndef __BH2011SoccerSymbols_h_
#define __BH2011SoccerSymbols_h_

#include "../../Symbols.h"
#include "Representations/BehaviorControl/BehaviorData.h"
#include "Representations/Perception/GoalPercept.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Modeling/BallModel.h"


/**
* \class BH2011SoccerSymbols
* The Xabsl symbols that are defined in "soccer_symbols.xabsl"
* \author Colin Graf
*/ 
class BH2011SoccerSymbols : public Symbols
{
public:
  /**
  * Constructor.
  */
  BH2011SoccerSymbols(BehaviorData& behaviorData, const GoalPercept& goalPercept, const RobotPose& robotPose, const FrameInfo& frameInfo, const FieldDimensions& fieldDimensions, const BallModel& ballModel) :
      behaviorData(behaviorData),
      robotPose(robotPose),
      goalPercept(goalPercept),
      frameInfo(frameInfo),
      fieldDimensions(fieldDimensions),
      ballModel(ballModel),
      disablePreInitialState(false)
  {
  }


  /** registers the symbols at an engine */
  void registerSymbols(xabsl::Engine& engine);

  /** updates the symbols */
  void update();

  /** initialize the symbols */
  void init();

float SoccerOpponentGoalAngle(){return soccerOpponentGoalAngle();}
float soccerOpponentGoalAngle();
float computePositionNextBallX();
float computePositionNextBallY();
float computeAngleNextBall();
float computePositionBehindBallX();
float computePositionBehindBallY();
float computePositionBehindBallAngle();
Vector2<float> computePositionBehindBall();
Vector2<float> computePosition();

float computeKeeperPoseY();

private:
  BehaviorData& behaviorData;
  const RobotPose& robotPose;
  const GoalPercept& goalPercept;
  const FrameInfo& frameInfo;
  const FieldDimensions& fieldDimensions;
  const BallModel& ballModel;

  bool disablePreInitialState;
  bool sideLeft;

  //The following only used by keeper.

  Vector2<float> M;
  //vertex A(x1,y1)
  Vector2<float> goalPostLeft;
  //vertex B(x2,y2)
  Vector2<float> ball; //ball.on_field;
  //vertex C(x3,y3)
  Vector2<float> goalPostRight;
  float clipMinX;
  float clipMaxX;
  float clipMinY;
  float clipMaxY;
  float clipMinAngle;
  float clipMaxAngle;
  bool desiredKeeperPoseReached;
  //Desired keeper position-x.
  float getDesiredKeeperPoseX();
  //Desired keeper position-y.
  float getDesiredKeeperPoseY();
  float getDesiredKeeperPoseAngle();
  void isDesiredKeeperPoseReached();
};


#endif // __BH2011SoccerSymbols_h_

