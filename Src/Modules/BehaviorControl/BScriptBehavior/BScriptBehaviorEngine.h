/**
* @file BScriptBehaviorEngine.h
* Implementation of class BehaviorControl.
* @author jeff
*/

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/KeyStates.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Infrastructure/TeamInfo.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/BehaviorControl/KickInfo.h"
#include "Representations/BehaviorControl/BehaviorLEDRequest.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/RobotsModel.h"
#include "Representations/Modeling/ObstacleModel.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/BallHypotheses.h"
#include "Representations/Modeling/FallDownState.h"
#include "Representations/Perception/GoalPercept.h"
#include "Representations/BehaviorControl/BehaviorControlOutput.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/MotionControl/HeadMotionRequest.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/Sensing/TorsoMatrix.h"

#include "../Util/b-script/src/bScriptEngine.h"
#include "BehaviorOutput.h"
#include "BehaviorInput.h"

MODULE(BScriptBehaviorEngine)
  REQUIRES(FrameInfo)
  REQUIRES(GameInfo)
  REQUIRES(RobotInfo)
  REQUIRES(OwnTeamInfo)
  REQUIRES(OpponentTeamInfo)
  REQUIRES(FieldDimensions)
  REQUIRES(RobotPose)
  REQUIRES(BallModel)
  REQUIRES(BallHypotheses)
  REQUIRES(ObstacleModel)
  REQUIRES(RobotsModel)
  REQUIRES(KeyStates)
  REQUIRES(MotionInfo)
  REQUIRES(FallDownState)
  REQUIRES(KickInfo)
  REQUIRES(GoalPercept)
  REQUIRES(RobotModel)
  REQUIRES(RobotDimensions)
  REQUIRES(TorsoMatrix)

  PROVIDES(HeadMotionRequest)
  REQUIRES(HeadMotionRequest)
  PROVIDES_WITH_MODIFY_AND_OUTPUT_AND_DRAW(MotionRequest)
  PROVIDES(SoundRequest)
  PROVIDES(BehaviorLEDRequest)
  PROVIDES(BehaviorControlOutput)
  //dummy to satisfy module manager
  PROVIDES(KickInfo)
END_MODULE

/**
* @class BScriptBehaviorEngine
*/
class BScriptBehaviorEngine : public BScriptBehaviorEngineBase
{
public:
  /**
  * Constructor.
  */
  BScriptBehaviorEngine();
  ~BScriptBehaviorEngine();

private:
  BScriptEngine engine;
  bool engineInitialized;
  BehaviorOutput output;
  BehaviorInput input;
  InputRepresentations* inputRepresentations;
#ifndef RELEASE
  std::string mainCallCode, lastMainCallCode;
  bool useModuleLibrary, lastUseModuleLibrary;
#endif

  void update(MotionRequest& motionRequest);
  void update(HeadMotionRequest& headMotionRequest);
  void update(SoundRequest& soundRequest);
  void update(BehaviorLEDRequest& behaviorLEDRequest);
  void update(BehaviorControlOutput& behaviorControlOutput);

  //dummy to satisfy module manager
  void update(KickInfo& kickInfo) {}

  bool initEngine();
  void init();

  int drawCallTree(const CallTreeNode& node, std::string indent = "", int yPos = 2000);
};

