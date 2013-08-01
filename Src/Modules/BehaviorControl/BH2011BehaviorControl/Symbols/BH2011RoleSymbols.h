/** 
* \file BH2011RoleSymbols.h
* Collection of symbols for dynamic role choosing.
* \author Colin Graf
*/

#ifndef __BH2011RoleSymbols_h_
#define __BH2011RoleSymbols_h_

#include "Modules/BehaviorControl/Symbols.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Infrastructure/TeamInfo.h"
#include "Representations/Infrastructure/TeamMateData.h"
#include "Representations/Modeling/BallModel.h"

/**
* \class BH2011RoleSymbols
* The Xabsl symbols that are defined in "role_symbols.xabsl"
* \author Colin Graf
*/ 
class BH2011RoleSymbols : public Symbols
{
public:
  /*
  * Constructor.
  * \param robotInfo A reference to the RobotInfo.
  */
  BH2011RoleSymbols(BehaviorData& behaviorData, 
					const RobotInfo& robotInfo, 
					const BallModel& ballModel, 
					const TeamMateData& teamMateData) : 
      behaviorData(behaviorData),
      robotInfo(robotInfo),
      ballModel(ballModel),
	  teamMateData(teamMateData),
      nativeRole(BehaviorData::undefined),
      isPenaltyStriker(false),
      isPenaltyKeeper(false),
	  NumOfConnectedPlayers(-1),
	  WasConnected(false)
  {
  }

  /** registers the symbols at an engine */
  void registerSymbols(xabsl::Engine& engine);

  /** updates the symbols */
  void update();

  /** initialize the symbols */
  void init();

private:
  BehaviorData& behaviorData;
  const RobotInfo& robotInfo;
  const BallModel& ballModel;
  const TeamMateData& teamMateData;

  BehaviorData::Role nativeRole; /**< The role defined by player number. */

  // Flags for striker
  bool isPenaltyStriker; /**< A flag to determine whether this striker is in the penalty shootout or not */

  // Flags for Keeper
  bool isPenaltyKeeper; /**< A flag to determine whether this keeper is in the penalty shootout or not */

  float NumOfConnectedPlayers;

  bool WasConnected;

  int getRobotModel();

  float getConnectedPlayers();

  bool getLostConnection();
};


#endif // __BH2011RoleSymbols_h_

