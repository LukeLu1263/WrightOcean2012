/** 
* \file BH2011TeamSymbols.cpp
* Implementation of symbols for TeamMateData.
* \author Colin Graf
* \author Katharina Gillmann
*/

#include "BH2011TeamSymbols.h"
#include "BH2011BallSymbols.h"
#include "BH2011LocatorSymbols.h"
#include <limits>

void BH2011TeamSymbols::registerSymbols(xabsl::Engine& engine)
{
  // team mate enumeration
  engine.registerEnumElement("team.mate", "team.mate.player1", 1);
  engine.registerEnumElement("team.mate", "team.mate.keeper", 1); 
  engine.registerEnumElement("team.mate", "team.mate.player2", 2);
  engine.registerEnumElement("team.mate", "team.mate.player3", 3);
  engine.registerEnumElement("team.mate", "team.mate.player4", 4);
  
  // team ball symbols
  engine.registerDecimalInputSymbol("team.ball.distance",this, &BH2011TeamSymbols::getBallDistance);
  engine.registerDecimalInputSymbolEnumeratedParameter("team.ball.distance", "team.ball.distance.player", "team.mate", &player);

  engine.registerDecimalInputSymbol("team.distance.to", this, &BH2011TeamSymbols::distanceTo);
  engine.registerDecimalInputSymbolEnumeratedParameter("team.distance.to", "team.distance.to.player", "team.mate", &player);
  engine.registerDecimalInputSymbolDecimalParameter("team.distance.to", "team.distance.to.x", &xPosition);
  engine.registerDecimalInputSymbolDecimalParameter("team.distance.to", "team.distance.to.y", &yPosition);
  
  engine.registerDecimalInputSymbol("team.ball.angle", this, &BH2011TeamSymbols::getBallAngle);
  engine.registerDecimalInputSymbolEnumeratedParameter("team.ball.angle", "team.ball.angle.player", "team.mate", &player);
  
  engine.registerDecimalInputSymbol("team.ball.x", this, &BH2011TeamSymbols::getBallX);
  engine.registerDecimalInputSymbolEnumeratedParameter("team.ball.x", "team.ball.x.player", "team.mate", &player);
  
  engine.registerDecimalInputSymbol("team.ball.y", this, &BH2011TeamSymbols::getBallY);
  engine.registerDecimalInputSymbolEnumeratedParameter("team.ball.y", "team.ball.y.player", "team.mate", &player);

  engine.registerDecimalInputSymbol("team.ball.position.field.x", this, &BH2011TeamSymbols::getBallPositionFieldX);
  engine.registerDecimalInputSymbolEnumeratedParameter("team.ball.position.field.x", "team.ball.position.field.x.player", "team.mate", &player);
  
  engine.registerDecimalInputSymbol("team.ball.position.field.y", this, &BH2011TeamSymbols::getBallPositionFieldY);
  engine.registerDecimalInputSymbolEnumeratedParameter("team.ball.position.field.y", "team.ball.position.field.y.player", "team.mate", &player);

  // team locator symbols
  engine.registerDecimalInputSymbol("team.locator.pose.x", this, &BH2011TeamSymbols::getLocatorPoseX);
  engine.registerDecimalInputSymbolEnumeratedParameter("team.locator.pose.x", "team.locator.pose.x.player", "team.mate", &player);
  
  engine.registerDecimalInputSymbol("team.locator.pose.y", this, &BH2011TeamSymbols::getLocatorPoseY);
  engine.registerDecimalInputSymbolEnumeratedParameter("team.locator.pose.y", "team.locator.pose.y.player", "team.mate", &player);
  
  engine.registerDecimalInputSymbol("team.locator.pose.angle", this, &BH2011TeamSymbols::getLocatorPoseAngle);
  engine.registerDecimalInputSymbolEnumeratedParameter("team.locator.pose.angle", "team.locator.pose.angle.player", "team.mate", &player);

  // tean role symbols
  engine.registerEnumeratedInputSymbol("team.role", "role.role", this, &BH2011TeamSymbols::getRole);
  engine.registerEnumeratedInputSymbolEnumeratedParameter("team.role", "team.role.player", "team.mate", &player);

  engine.registerEnumeratedInputSymbol("team.action", "soccer.behavior_action", this, &BH2011TeamSymbols::getAction);
  engine.registerEnumeratedInputSymbolEnumeratedParameter( "team.action", "team.action.player", "team.mate", &player);

  engine.registerBooleanInputSymbol("team.is_attacking", this, &BH2011TeamSymbols::isTeamAttacking);


  engine.registerEnumeratedInputSymbol("team.first_team_mate","team.mate", this, &BH2011TeamSymbols::getFirstTeamMate);
  engine.registerEnumeratedInputSymbol("team.second_team_mate","team.mate", this, &BH2011TeamSymbols::getSecondTeamMate);

  engine.registerDecimalInputSymbol("team.ball_distance_all_players",this, &BH2011TeamSymbols::getBallDistanceAllPlayers);
  engine.registerDecimalInputSymbol("team.ball_distance_team_mate_all_players", this, &BH2011TeamSymbols::getBallDistanceTeamMateAllPlayers);
  engine.registerDecimalInputSymbolEnumeratedParameter("team.ball_distance_team_mate_all_players", "team.ball_distance_team_mate_all_players.player", "team.mate", &player);

  engine.registerBooleanInputSymbol("team.kickoff_in_progress", &behaviorData.kickoffInProgress);
  engine.registerBooleanInputSymbolEnumeratedParameter("team.kickoff_in_progress", "team.kickoff_in_progress.player", "team.mate", &player);

  engine.registerDecimalInputSymbol("team.ball_position_all_players.x", this, &BH2011TeamSymbols::getBallPositionAllPlayersX);
  engine.registerDecimalInputSymbol("team.ball_position_all_players.y", this, &BH2011TeamSymbols::getBallPositionAllPlayersY);

}

float BH2011TeamSymbols::getBallDistance()
{
  if(player == robotInfo.number)
    return ballModel.estimate.getDistance();
  if(player >= TeamMateData::firstPlayer && player < TeamMateData::numOfPlayers)
    return teamMateData.ballModels[player].estimate.getDistance();
  return 0;
}

float BH2011TeamSymbols::distanceTo()
{
  if(player == robotInfo.number)
	  return (Pose2D(0.f, xPosition, yPosition) - robotPose).translation.abs();
  //  return BH2011LocatorSymbols::getDistanceTo();
  if(player >= TeamMateData::firstPlayer && player < TeamMateData::numOfPlayers)
    return Geometry::distanceTo(teamMateData.robotPoses[player], Vector2<float>(xPosition, yPosition));
  return 0;
}



float BH2011TeamSymbols::getBallAngle()
{
  if(player == robotInfo.number)
    return toDegrees(ballModel.estimate.getAngle());
  if(player >= TeamMateData::firstPlayer && player < TeamMateData::numOfPlayers)
    return toDegrees(teamMateData.ballModels[player].estimate.getAngle());
  return 0;
}

float BH2011TeamSymbols::getBallX()
{
  if(player == robotInfo.number)
	  return ballModel.estimate.position.x;
  if(player >= TeamMateData::firstPlayer && player < TeamMateData::numOfPlayers)
    return teamMateData.ballModels[player].estimate.position.x;
  return 0;
}

float BH2011TeamSymbols::getBallY()
{
  if(player == robotInfo.number)
    return ballModel.estimate.position.y;
  if(player >= TeamMateData::firstPlayer && player < TeamMateData::numOfPlayers)
    return teamMateData.ballModels[player].estimate.position.y;
  return 0;
}

float BH2011TeamSymbols::getBallPositionFieldX()
{
  if(player == robotInfo.number)
	 return ballModel.estimate.position.x;
   // return BH2011BallSymbols::getBallPositionRobotX();
  if(player >= TeamMateData::firstPlayer && player < TeamMateData::numOfPlayers)
    return teamMateData.ballModels[player].estimate.getPositionInFieldCoordinates(teamMateData.robotPoses[player]).x;
  return 0;
}

float BH2011TeamSymbols::getBallPositionFieldY()
{
  if(player == robotInfo.number)
    return ballModel.estimate.position.y;
  if(player >= TeamMateData::firstPlayer && player < TeamMateData::numOfPlayers)
    return teamMateData.ballModels[player].estimate.getPositionInFieldCoordinates(teamMateData.robotPoses[player]).y;
  return 0;
}

float BH2011TeamSymbols::getLocatorPoseX()
{
  if(player == robotInfo.number)
    return robotPose.translation.x;
  if(player >= TeamMateData::firstPlayer && player < TeamMateData::numOfPlayers)
    return teamMateData.robotPoses[player].translation.x;
  return 0;
}

float BH2011TeamSymbols::getLocatorPoseY()
{
  if(player == robotInfo.number)
    return robotPose.translation.y;
  if(player >= TeamMateData::firstPlayer && player < TeamMateData::numOfPlayers)
    return teamMateData.robotPoses[player].translation.y;
  return 0;
}

float BH2011TeamSymbols::getLocatorPoseAngle()
{
  if(player == robotInfo.number)
    return toDegrees(robotPose.getAngle());
  if(player >= TeamMateData::firstPlayer && player < TeamMateData::numOfPlayers)
    return toDegrees(teamMateData.robotPoses[player].getAngle());
  return 0;
}

float BH2011TeamSymbols::getConnectedPlayers()
{
  return (float)teamMateData.numOfConnectedPlayers;
}

int BH2011TeamSymbols::getFirstTeamMate()
{
  return teamMateData.firstTeamMate;
}

int BH2011TeamSymbols::getSecondTeamMate()
{
  return teamMateData.secondTeamMate;
}

bool BH2011TeamSymbols::getLostConnection()
{
  return teamMateData.numOfConnectedPlayers <= 0 && teamMateData.wasConnected;
}

int BH2011TeamSymbols::getRole()
{
  if(player >= TeamMateData::firstPlayer && player < TeamMateData::numOfPlayers)
    return teamMateData.behaviorData[player].role;
  return BehaviorData::undefined;
}

int BH2011TeamSymbols::getAction()
{
  if(player >= TeamMateData::firstPlayer && player < TeamMateData::numOfPlayers)
    return teamMateData.behaviorData[player].action;
  return BehaviorData::unknown;
}

bool BH2011TeamSymbols::isTeamAttacking()
{
  if(frameInfo.getTimeSince(ballModel.timeWhenLastSeen) < 2000)
  {
    if(ballModel.estimate.getPositionInFieldCoordinates(robotPose).x > -500.)
      return true;
  }
  else
  {
    for(int i = TeamMateData::firstPlayer; i < TeamMateData::numOfPlayers; ++i)
    //  if(teamMateData.behaviorData[i].role == BehaviorData::striker)
    //I think you wanted to test the timestamp here or?
     //if(frameInfo.getTimeSince(teamMateData.timeStamps[i] && 
     //   teamMateData.ballModels[i].estimate.getPositionInFieldCoordinates(teamMateData.robotPoses[i]).x > 0.)
        if(frameInfo.getTimeSince(teamMateData.ballModels[i].timeWhenLastSeen) < 2000 && teamMateData.ballModels[i].estimate.getPositionInFieldCoordinates(teamMateData.robotPoses[i]).x > -500.)
          return true;
  }


//This would make when I sometime had the ball and not seen rolling away, return true
 /* if(behaviorData.ballHold)
    return true;

//This would make, that when at sometime in the past someone of the robots had somehow the ball and when he never saw it somewhere else far away, then return
true
  for(int i = TeamMateData::firstPlayer; i < TeamMateData::numOfPlayers; ++i)
    if(teamMateData.timeStamps[i] && teamMateData.behaviorData[i].ballHold)
      return true;*/
  return false;
}

float BH2011TeamSymbols::getBallDistanceAllPlayers()
{
  if(frameInfo.getTimeSince(ballModel.timeWhenLastSeen) < 5000 || teamMateData.numOfConnectedPlayers == 0)
    return ballModel.estimate.getDistance();
  else 
  {
    int minTimeSinceBallSeen = std::numeric_limits<int>::max();
    int teamMate = 0;
    for(int i = TeamMateData::firstPlayer; i < TeamMateData::numOfPlayers; i++)
    {
      if(frameInfo.getTimeSince(teamMateData.timeStamps[i]) < networkTimeout && frameInfo.getTimeSince(teamMateData.ballModels[i].timeWhenLastSeen) < minTimeSinceBallSeen)
      {
        minTimeSinceBallSeen = frameInfo.getTimeSince(teamMateData.ballModels[i].timeWhenLastSeen);
        teamMate = i;
      }
    }
    ASSERT(teamMate >=TeamMateData::firstPlayer && teamMate < TeamMateData::numOfPlayers);
    return Geometry::fieldCoord2Relative(robotPose, teamMateData.ballModels[teamMate].estimate.getPositionInFieldCoordinates(teamMateData.robotPoses[teamMate])).abs();
  }
}

Vector2<float> BH2011TeamSymbols::computeBallPositionAllPlayers()
{
  if(frameInfo.getTimeSince(ballModel.timeWhenLastSeen) < 5000 || teamMateData.numOfConnectedPlayers == 0)
    return ballModel.lastPerception.getPositionInFieldCoordinates(robotPose);
  else 
  {
    int minTimeSinceBallSeen = std::numeric_limits<int>::max();
    int teamMate = 0;
    for(int i = TeamMateData::firstPlayer; i < TeamMateData::numOfPlayers; i++)
    {
      if(frameInfo.getTimeSince(teamMateData.timeStamps[i]) < networkTimeout && frameInfo.getTimeSince(teamMateData.ballModels[i].timeWhenLastSeen) < minTimeSinceBallSeen)
      {
        minTimeSinceBallSeen = frameInfo.getTimeSince(teamMateData.ballModels[i].timeWhenLastSeen);
        teamMate = i;
      }
    }
    ASSERT(teamMate >=TeamMateData::firstPlayer && teamMate < TeamMateData::numOfPlayers);
    return teamMateData.ballModels[teamMate].lastPerception.getPositionInFieldCoordinates(teamMateData.robotPoses[teamMate]);
  }
} 

  float BH2011TeamSymbols::getBallPositionAllPlayersX()
  {
    return computeBallPositionAllPlayers().x;
  }

  float BH2011TeamSymbols::getBallPositionAllPlayersY()
  {
    return computeBallPositionAllPlayers().y;
  }

float BH2011TeamSymbols::getBallDistanceTeamMateAllPlayers()
{
  if(frameInfo.getTimeSince(teamMateData.ballModels[player].timeWhenLastSeen) < 5000)
    return teamMateData.ballModels[player].estimate.getDistance();
  else
  {
    int minTimeSinceBallSeen = std::numeric_limits<int>::max();
    int teamMate = 0;
    for(int i = TeamMateData::firstPlayer; i < TeamMateData::numOfPlayers; i++)
    {
      if(i != player)
      {
        if(i != robotInfo.number)
        {
          if(frameInfo.getTimeSince(teamMateData.timeStamps[i]) < networkTimeout && frameInfo.getTimeSince(teamMateData.ballModels[i].timeWhenLastSeen) < minTimeSinceBallSeen)
          {
           minTimeSinceBallSeen = frameInfo.getTimeSince(teamMateData.ballModels[i].timeWhenLastSeen);
           teamMate = i;
          }
        }
        else
        {
          if(frameInfo.getTimeSince(ballModel.timeWhenLastSeen) < minTimeSinceBallSeen)
          {
            minTimeSinceBallSeen = frameInfo.getTimeSince(ballModel.timeWhenLastSeen);
            teamMate = i;
          }
        }
      }
    }
      if(teamMate != robotInfo.number)
        return Geometry::fieldCoord2Relative(teamMateData.robotPoses[player], teamMateData.ballModels[teamMate].estimate.getPositionInFieldCoordinates(teamMateData.robotPoses[teamMate])).abs();
      else
        return Geometry::fieldCoord2Relative(teamMateData.robotPoses[player], ballModel.estimate.getPositionInFieldCoordinates(robotPose)).abs();
  }
}
