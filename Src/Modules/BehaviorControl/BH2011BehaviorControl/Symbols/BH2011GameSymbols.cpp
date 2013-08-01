/**
* @file BH2011GameSymbols.cpp
*
* Implementation of class BH2011GameSymbols.
*
* @author Judith Müller
*/

#include "BH2011GameSymbols.h"
#include "Tools/Settings.h"

void BH2011GameSymbols::registerSymbols(xabsl::Engine& engine)
{
  engine.registerEnumElement("game.team_color", "game.team_color.blue", TEAM_BLUE);
  engine.registerEnumElement("game.team_color", "game.team_color.red", TEAM_RED);
  engine.registerEnumElement("game.team_color", "game.team_color.none", TEAM_NONE);

  engine.registerEnumElement("game.state", "game.state._initial", STATE_INITIAL);
  engine.registerEnumElement("game.state", "game.state.ready", STATE_READY);
  engine.registerEnumElement("game.state", "game.state.set", STATE_SET);
  engine.registerEnumElement("game.state", "game.state.playing", STATE_PLAYING);
  engine.registerEnumElement("game.state", "game.state.finished", STATE_FINISHED);

  engine.registerEnumElement("game.secondary_state", "game.secondary_state.normal", STATE2_NORMAL);
  engine.registerEnumElement("game.secondary_state", "game.secondary_state.penaltyshoot", STATE2_PENALTYSHOOT);

  engine.registerEnumElement("game.penalty", "game.penalty.none", PENALTY_NONE);
  engine.registerEnumElement("game.penalty", "game.penalty.ball_holding", PENALTY_SPL_BALL_HOLDING);
  engine.registerEnumElement("game.penalty", "game.penalty.player_pushing", PENALTY_SPL_PLAYER_PUSHING);
  engine.registerEnumElement("game.penalty", "game.penalty.obstruction", PENALTY_SPL_OBSTRUCTION);
  engine.registerEnumElement("game.penalty", "game.penalty.inactive_player", PENALTY_SPL_INACTIVE_PLAYER);
  engine.registerEnumElement("game.penalty", "game.penalty.illegal_defender", PENALTY_SPL_ILLEGAL_DEFENDER);
  engine.registerEnumElement("game.penalty", "game.penalty.leaving_the_field", PENALTY_SPL_LEAVING_THE_FIELD);
  engine.registerEnumElement("game.penalty", "game.penalty.playing_with_hands", PENALTY_SPL_PLAYING_WITH_HANDS);
  engine.registerEnumElement("game.penalty", "game.penalty.request_for_pickup", PENALTY_SPL_REQUEST_FOR_PICKUP);
  engine.registerEnumElement("game.penalty", "game.penalty.manual", PENALTY_MANUAL);

  engine.registerEnumeratedOutputSymbol("game.team_color", "game.team_color",
                                        this, &BH2011GameSymbols::setOwnTeamColor, &BH2011GameSymbols::getOwnTeamColor);

  engine.registerEnumeratedInputSymbol("game.opponent_team_color", "game.team_color", this, &BH2011GameSymbols::getOppTeamColor);

  engine.registerEnumeratedInputSymbol("game.player_number", "role.role", this, &BH2011GameSymbols::getPlayerNumber);

  engine.registerEnumeratedOutputSymbol("game.penalty", "game.penalty",
                                        this, &BH2011GameSymbols::setPenalty, &BH2011GameSymbols::getPenalty);

  engine.registerDecimalInputSymbol("game.penalty.remaining_time",
                                    this, &BH2011GameSymbols::getPenaltyRemainingTime);

  engine.registerEnumeratedOutputSymbol("game.state", "game.state",
                                        this, &BH2011GameSymbols::setState , &BH2011GameSymbols::getState);

  engine.registerEnumeratedOutputSymbol("game.kickoff_team", "game.team_color",
                                        this, &BH2011GameSymbols::setKickoffTeam, &BH2011GameSymbols::getKickoffTeam);

  engine.registerEnumeratedOutputSymbol("game.secondary_state", "game.secondary_state",
                                        this, &BH2011GameSymbols::setSecondaryState, &BH2011GameSymbols::getSecondaryState);

  engine.registerDecimalInputSymbol("game.own_score", this, &BH2011GameSymbols::getOwnScore);

  engine.registerDecimalInputSymbol("game.remaining_time", this, &BH2011GameSymbols::getRemainingTime);

  engine.registerDecimalInputSymbol("game.time_since_playing_started", this, &BH2011GameSymbols::getTimeSincePlayingState);

  engine.registerEnumeratedOutputSymbol("game.drop_in_team", "game.team_color", this, &BH2011GameSymbols::setDropInTeam, &BH2011GameSymbols::getDropInTeam);
  engine.registerDecimalOutputSymbol("game.drop_in_time", this, &BH2011GameSymbols::setDropInTime, &BH2011GameSymbols::getDropInTime);

  engine.registerDecimalInputSymbol("game.time_since_last_penalization", this, &BH2011GameSymbols::getTimeSinceLastPenalization);
  engine.registerDecimalInputSymbol("game.time_since_initial", this, &BH2011GameSymbols::getTimeSinceInitial);
  engine.registerDecimalInputSymbol("game.time_since_last_package_received", this, &BH2011GameSymbols::getTimeSinceLastPackageReceived);

  engine.registerBooleanInputSymbol("game.disable_pre_initial", &disablePreInitialState);
  engine.registerDecimalInputSymbol("game.pn",this, &BH2011GameSymbols::GetPN);
  engine.registerBooleanInputSymbol("game.kickoff_in_progress", &behaviorData.kickoffInProgress);
}

void BH2011GameSymbols::init()
{
  lastGameState = gameInfo.state;
  timeSincePen2Play = 0;
#ifdef TARGET_SIM
  disablePreInitialState = true;
#else
  disablePreInitialState = Global::getSettings().recover;
#endif
}

void BH2011GameSymbols::update()
{
  if(gameInfo.state == STATE_INITIAL)
  {
    lastTimeInInitial = frameInfo.time;
  }
  else if(gameInfo.state == STATE_PLAYING && lastGameState != STATE_PLAYING)
  {
    timeWhenStartedPlaying = frameInfo.time;
	behaviorData.kickoffInProgress =  gameInfo.kickOffTeam == ownTeamInfo.teamColor;
    counter = 0;
  }
  lastGameState = gameInfo.state;
  if(behaviorData.kickoffInProgress && frameInfo.getTimeSince(ballModel.timeWhenLastSeen) == 0)
  {
    if(ballModel.lastPerception.getPositionInFieldCoordinates(robotPose).abs() > 700)
      counter++;
    else if(counter != 0)
      counter--;
    if(counter >= 10)
     behaviorData.kickoffInProgress = false;
  }
  if(robotInfo.secsTillUnpenalised > 0)
  {
    timeSincePen2Play = frameInfo.time;
  }
}
