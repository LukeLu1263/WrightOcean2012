/** 
* \file BH2011TeamSymbols.h
* Implementation of symbols for TeamMateData.
* \author Colin Graf
*/

#ifndef __BH2011TeamSymbols_h_
#define __BH2011TeamSymbols_h_

#include "Modules/BehaviorControl/Symbols.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/TeamMateData.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Configuration/FieldDimensions.h"

/**
* \class BH2011TeamSymbols
* The Xabsl symbols that are defined in "team_symbols.xabsl"
* \author Colin Graf
*/ 
class BH2011TeamSymbols : public Symbols
{
public:
  /**
  * Constructor.
  * \param robotInfo A reference to the RobotInfo.
  * \param robotPose A reference to the RobotPose.
  * \param ballModel A reference to the BallModel.
  * \param teamMateData A refernece to the TeamMateData.
  * \param fieldDimensions A reference to the FieldDimensions.
  * \param frameInfo a reference to the FrameInfo.
  * \param behaviorData A reference to the BehaviorData.
  */
  BH2011TeamSymbols(
    const RobotInfo& robotInfo, 
    const RobotPose& robotPose,
    const BallModel& ballModel,
    const TeamMateData& teamMateData,
    const FieldDimensions& fieldDimensions,
    const FrameInfo& frameInfo,
    const BehaviorData& behaviorData) : 
      robotInfo(robotInfo),
      ballModel(ballModel),
      robotPose(robotPose),
      teamMateData(teamMateData),
      fieldDimensions(fieldDimensions),
      frameInfo(frameInfo),
      behaviorData(behaviorData)
  {
  }


  const static int networkTimeout = 2000;

  /** registers the symbols at an engine */
  void registerSymbols(xabsl::Engine& engine);

  /** updates the symbols */
  void update() {};
  Vector2<float>  staticComputeBallPositionAllPlayers() { return computeBallPositionAllPlayers();}

private:

  float getBallDistance();
  float distanceTo();
  float getBallAngle();
  float getBallX();
  float getBallY();
  float getBallPositionFieldX();
  float getBallPositionFieldY();
  float getLocatorPoseX();
  float getLocatorPoseY();
  float getLocatorPoseAngle();
  float getConnectedPlayers();
  int getFirstTeamMate();
  int getSecondTeamMate();
  bool getLostConnection();
  int getRole();
  int getAction();
  bool isTeamAttacking();
  float getBallDistanceAllPlayers();
  float getBallDistanceTeamMateAllPlayers();
  float getBallPositionAllPlayersX();
  float getBallPositionAllPlayersY();
  Vector2<float> computeBallPositionAllPlayers();


  // data sources
  const RobotInfo& robotInfo;
  const BallModel& ballModel;
  const RobotPose& robotPose;
  const TeamMateData& teamMateData;
  const FieldDimensions& fieldDimensions;
  const FrameInfo& frameInfo;
  const BehaviorData& behaviorData;

  // symbol parameter
  int player;
  float xPosition;
  float yPosition;


 // friend class BH2011SoccerSymbols; 
//  friend class BH2011BallSymbols;
};


#endif // __BH2011TeamSymbols_h_

