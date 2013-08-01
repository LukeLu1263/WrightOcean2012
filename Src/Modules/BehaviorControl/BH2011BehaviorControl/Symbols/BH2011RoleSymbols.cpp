/** 
* \file BH2011RoleSymbols.cpp
* Collection of symbols for dynamic role choosing.
* \author Colin Graf
*/

#include "BH2011RoleSymbols.h"
#include "BH2011GoalSymbols.h"
#include "BH2011BallSymbols.h"
#include "BH2011LocatorSymbols.h"

//#include "BH2011SoccerSymbols.h"
#include "Tools/Team.h"


void BH2011RoleSymbols::registerSymbols(xabsl::Engine& engine)
{
  engine.registerDecimalInputSymbol("role.connected_players", this, &BH2011RoleSymbols::getConnectedPlayers); //teamMateData
  engine.registerBooleanInputSymbol("role.lost_connection", this, &BH2011RoleSymbols::getLostConnection); //teamMateData
  engine.registerBooleanOutputSymbol("role.is_penalty_striker", &isPenaltyStriker);
  engine.registerBooleanOutputSymbol("role.is_penalty_keeper", &isPenaltyKeeper);

  engine.registerEnumElement("role.role", "role.role.undefined", BehaviorData::undefined); 
  engine.registerEnumElement("role.role", "role.role.keeper", BehaviorData::keeper);
  engine.registerEnumElement("role.role", "role.role.defender", BehaviorData::defender);
  engine.registerEnumElement("role.role", "role.role.supporter", BehaviorData::supporter);
  engine.registerEnumElement("role.role", "role.role.striker", BehaviorData::striker);
 

  /* role.is_localisation_limit_active, ¸ÃsymbolÎ´ÊµÏÖ*/
  engine.registerEnumeratedOutputSymbol("role.role", "role.role", (int*)&behaviorData.role);
  engine.registerEnumeratedInputSymbol("role.native_role", "role.role", (int*)&nativeRole);
}

void BH2011RoleSymbols::init() 
{
  if(robotInfo.number >= BehaviorData::firstRole && robotInfo.number < BehaviorData::numOfRoles)
    behaviorData.role = nativeRole = (BehaviorData::Role)robotInfo.number;
}

void BH2011RoleSymbols::update()
{
  // calculate
  const BallState& estimate = ballModel.estimate;
  Vector2 <float> ballPositionRel = estimate.position;
  
  NumOfConnectedPlayers = (float)teamMateData.numOfConnectedPlayers;
  WasConnected = teamMateData.wasConnected;
}

float BH2011RoleSymbols::getConnectedPlayers()
{
	return (float)teamMateData.numOfConnectedPlayers;
}

bool BH2011RoleSymbols::getLostConnection()
{
	return teamMateData.numOfConnectedPlayers <= 0 && teamMateData.wasConnected;
}