/**
* @file BH2011GameSymbols.h
* Declaration of class BH2011GameSymbols.
* @author Judith Müller
*/

#pragma once

#include "Representations/BehaviorControl/BehaviorData.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Infrastructure/TeamInfo.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Modeling/BallModel.h"
#include "Modules/BehaviorControl/Symbols.h"

/**
* The Xabsl symbols that are defined in "game_symbols.xabsl"
* @author Max Risler
* @author Judith Müller
*/
class BH2011GameSymbols : public Symbols
{
public:
  /*
  * Constructor.
  */
  BH2011GameSymbols(BehaviorData& behaviorData, RobotInfo& robotInfo, OwnTeamInfo& ownTeamInfo, GameInfo& gameInfo, const FrameInfo& frameInfo,const BallModel& ballModel, const RobotPose& robotPose) :
    behaviorData(behaviorData),
    robotInfo(robotInfo),
    ownTeamInfo(ownTeamInfo),
    gameInfo(gameInfo),
    frameInfo(frameInfo),
    timeWhenStartedPlaying(0),
    lastTimeInInitial(0),
	ballModel(ballModel),
    robotPose(robotPose)
  {}

private:
  int getOwnTeamColor() {return (int)ownTeamInfo.teamColor;}
  void setOwnTeamColor(int color) {ownTeamInfo.teamColor = color;}

  int getOppTeamColor() {return ((int)ownTeamInfo.teamColor == TEAM_RED) ? TEAM_BLUE : TEAM_RED;}

  int getPlayerNumber() {return robotInfo.number;}

  int getPenalty() {return (int)robotInfo.penalty;}
  void setPenalty(int penalty) {robotInfo.penalty = penalty;}

  float getPenaltyRemainingTime() {return (float)robotInfo.secsTillUnpenalised;}

  int getState() {return (int)gameInfo.state;}
  void setState(int state) {gameInfo.state = state;}

  int getKickoffTeam() {return (int)gameInfo.kickOffTeam;}
  void setKickoffTeam(int kickoff) {gameInfo.kickOffTeam = kickoff;}

  int getSecondaryState() {return (int)gameInfo.secondaryState;}
  void setSecondaryState(int secondaryState) {gameInfo.secondaryState = secondaryState;}

  float getOwnScore() {return (float)ownTeamInfo.score;}

  float getRemainingTime() {return (float)gameInfo.secsRemaining;}

  float getTimeSinceInitial() {return (float)frameInfo.getTimeSince(lastTimeInInitial);}

  float getTimeSincePlayingState() {return gameInfo.state !=  STATE_PLAYING ? 0.f : float(frameInfo.getTimeSince(timeWhenStartedPlaying));}

  float getTimeSinceLastPenalization() {return (float) frameInfo.getTimeSince(timeSincePen2Play);}

  float getTimeSinceLastPackageReceived() {return (float) gameInfo.timeSinceLastPackageReceived;}

  int getDropInTeam() {return gameInfo.dropInTime > 0 ? (int)gameInfo.dropInTeam : (int) TEAM_NONE;}
  void setDropInTeam(int dropInTeam) {gameInfo.dropInTeam = dropInTeam;}

  float getDropInTime() {return (float)gameInfo.dropInTime;}
  void setDropInTime(float dropInTime) {gameInfo.dropInTime = static_cast<RoboCup::uint16>(dropInTime);}
  float GetPN(){return (float) robotInfo.number;}

  BehaviorData& behaviorData;
  RobotInfo& robotInfo;
  OwnTeamInfo& ownTeamInfo;
  GameInfo& gameInfo;
  const FrameInfo& frameInfo;
  const BallModel& ballModel;
  const RobotPose& robotPose;

  unsigned char lastGameState;
  unsigned timeWhenStartedPlaying;
  unsigned timeSincePen2Play;

  unsigned int lastTimeInInitial;

  bool disablePreInitialState;

  /** registers the symbols at an engine */
  void registerSymbols(xabsl::Engine& engine);

  void update();

  void init();

  int counter;
};
