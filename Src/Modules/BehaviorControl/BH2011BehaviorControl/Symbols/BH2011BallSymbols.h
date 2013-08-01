/**
* @file BH2011BallSymbols.h
*
* Declaration of class BallSymbols.
*
* @author Max Risler
* @author Colin Graf
*/

#pragma once

#include "Modules/BehaviorControl/Symbols.h"
#include "Representations/BehaviorControl/BehaviorData.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/BallHypotheses.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Infrastructure/LEDRequest.h"
#include "Representations/Infrastructure/TeamMateData.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Modeling/RobotsModel.h"
#include "Representations/Infrastructure/TeamInfo.h"
#include "Representations/Sensing/TorsoMatrix.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/Configuration/RobotDimensions.h"
//#include "Representations/Infrastructure/HeadMotionInfo.h"

#define BallGridSizeX 60
#define BallGridSizeY 40


/**
* The Xabsl symbols that are defined in "ball_symbols.xabsl"
*/
class BH2011BallSymbols : public Symbols
{
public:
  /** Constructor. */
  BH2011BallSymbols(const RobotInfo& robotInfo,
                    const BallModel& ballModel,
                    const BallHypotheses& ballHypotheses,
                    const FrameInfo& frameInfo,
                    const RobotPose& robotPose,
                    const TeamMateData& teamMateData,
                    const FieldDimensions& fieldDimensions,
                    const RobotsModel& robotsModel,
                    const OwnTeamInfo& ownTeamInfo,
                    const TorsoMatrix& torsoMatrix,
                    const RobotModel& robotModel,
                    const RobotDimensions& robotDimensions):
    robotInfo(robotInfo),
    frameInfo(frameInfo),
    ballModel(ballModel),
    ballHypotheses(ballHypotheses),
    robotPose(robotPose),
    teamMateData(teamMateData),
    fieldDimensions(fieldDimensions),
    robotsModel(robotsModel),
    ownTeamInfo(ownTeamInfo),
    torsoMatrix(torsoMatrix),
    robotModel(robotModel),
    robotDimensions(robotDimensions),
    ballWasSeen(false),
    ballPositionRel(0, 0),
    ballEndPositionRel(0, 0),
    ballPositionField(0, 0),
    ballEndPositionField(0, 0),
    ballSpeedRel(0, 0),
    ballSpeedField(0, 0),
    ballKickRel(0, 0),
	countSupporterSwitch(0),
    timeSinceBallWasSeen(10000.0)
  {
    memset(teamBallPositionsRelTime, 0, sizeof(teamBallPositionsRelTime));
    memset(teamBallPositionsFieldTime, 0, sizeof(teamBallPositionsFieldTime));
		//
	memset(ballGrid, 0, sizeof(ballGrid));
  }

  /** registers the symbols at an engine */
  void registerSymbols(xabsl::Engine& engine);

  /** updates the symbols */
  void update();

  float getBallAngle();
  float getBallDistance();
  float getBallPositionRobotX();
  float getBallPositionRobotY();

private:
  const RobotInfo& robotInfo;
  const FrameInfo& frameInfo;
  const BallModel& ballModel;
  const BallHypotheses& ballHypotheses;
  const RobotPose& robotPose;
  const TeamMateData& teamMateData;
  const FieldDimensions& fieldDimensions;
  const RobotsModel& robotsModel;
  const OwnTeamInfo& ownTeamInfo;
  const TorsoMatrix& torsoMatrix;
  const RobotModel& robotModel;
  const RobotDimensions& robotDimensions;
//  const HeadMotionInfo& headMotionInfo;

  bool ballWasSeen;
  Vector2 <> ballPositionRel;
  Vector2 <> ballEndPositionRel;
  Vector2 <> seenBallPositionRel;
  Vector2 <> ballPositionField;
  Vector2 <> ballEndPositionField;
  Vector2 <> ballSpeedRel;
  Vector2 <> ballSpeedField;
  Vector2 <> ballLastSeenEstimate;
  Vector2 <> ballPositionLastSeenEstimate;
  Vector2 <> ballKickRel;

  unsigned teamBallPositionsRelTime[TeamMateData::numOfPlayers];
  Vector2 <> teamBallPositionsRel[TeamMateData::numOfPlayers];
  unsigned teamBallPositionsFieldTime[TeamMateData::numOfPlayers];
  Vector2 <> teamBallPositionsField[TeamMateData::numOfPlayers];

  float getBallFieldRobotX();
  float getBallFieldRobotY();
  float getBallSeenDistance();
  float getSeenBallAngle();
  float getYPosWhenBallReachesOwnYAxis();
  float getTimeWhenBallReachesOwnYAxis();
  float getBallLastSeenEstimateX();
  float getBallLastSeenEstimateY();
  float getBallPositionLastSeenEstimateX();
  float getBallPositionLastSeenEstimateY();

  float getBallDistanceToOwnGoal();

  float getOnFieldX();
  float getOnFieldY();
  float getOnFieldAngle();
  float getOnFieldSpeedX();
  float getOnFieldSpeedY();
  float getTimeSinceDisappeared();

  Vector2 <> calculateBallInRobotOrigin(const Vector2 <>& ballRel);
  float timeSinceBallWasSeen;


    //
  bool isNearestToBallLast;
  int countSwitch;
  int countSupporterSwitch;
  unsigned timeTMateBallLast;

  // Role switch for 4 players
  bool isPlayerValid[TeamMateData::numOfPlayers];
  double distToBallTMate[TeamMateData::numOfPlayers];
  float strikerPn;
  float offensiveSupporterPn;
  float defensiveSupporterPn;

  void updateRole();
  void updateStrikerPn();
  //

  bool isNearestToBall;
  bool getIsNearestToBall();
  bool GetIsNearestToBall() {return getIsNearestToBall(); };


  //
  int ballGrid[BallGridSizeX][BallGridSizeY];
  unsigned ballTMateTime[TeamMateData::numOfPlayers];
  unsigned poseTMateTime[TeamMateData::numOfPlayers];
  RobotPose poseTMate[TeamMateData::numOfPlayers];
  //HeadMotionInfo headTMate[TeamMateData::numOfPlayers];
  
  bool ballWasSeenTeam;
  Vector2 <> ballPositionTeamField;
  bool updateBallTeam();

  bool isStopCountingTime;

  bool someoneIsKicking;



};
