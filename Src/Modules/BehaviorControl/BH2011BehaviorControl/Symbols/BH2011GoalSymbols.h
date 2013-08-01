/**
 * @file BH2011GoalSymbols.h
 *
 * Declaration of class BH2011GoalSymbols.
 *
 * @author Tim Laue
 * @author Carsten Koenemann
 */

#pragma once

#include "Modules/BehaviorControl/Symbols.h"
#include "Representations/Modeling/FreePartOfOpponentGoalModel.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/FallDownState.h"
#include "Representations/Perception/GoalPercept.h"
#include "Representations/Sensing/GroundContactState.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Configuration/DamageConfiguration.h"


/**
 * The Xabsl symbols that are defined in "goal_symbols.xabsl"
 */
class BH2011GoalSymbols : public Symbols
{
public:
  BH2011GoalSymbols
  (
    const FreePartOfOpponentGoalModel& freePartOfOpponentGoalModel,
    const RobotPose& robotPose,
    const FallDownState& fallDownState,
    const GoalPercept& goalPercept,
    const GroundContactState& groundContactState,
    const GameInfo& gameInfo,
    const FrameInfo& frameInfo,
    const FieldDimensions& fieldDimensions,
    const DamageConfiguration& damageConfiguration
  ):
    freePartOfOpponentGoalModel(freePartOfOpponentGoalModel),
    robotPose(robotPose),
    fallDownState(fallDownState),
    goalPercept(goalPercept),
    groundContactState(groundContactState),
    gameInfo(gameInfo),
    frameInfo(frameInfo),
    fieldDimensions(fieldDimensions),
    damageConfiguration(damageConfiguration)
  {}


private:
  /** A reference to the FreePartOfOpponentGoalModel */
  const FreePartOfOpponentGoalModel& freePartOfOpponentGoalModel;

  /** A reference to the RobotPose */
  const RobotPose& robotPose;

  /** A reference to the FallDownState */
  const FallDownState& fallDownState;

  /** A reference to the GoalModel */
  const GoalPercept& goalPercept;

  /** A reference to the GroundContactState */
  const GroundContactState& groundContactState;

  /** A reference to the GameInfo */
  const GameInfo& gameInfo;

  /** A reference to the FrameInfo */
  const FrameInfo& frameInfo;

  /** A reference to the FieldDimensions */
  const FieldDimensions& fieldDimensions;

  /** A reference to the DamageConfiguration */
  const DamageConfiguration& damageConfiguration;

  /** registers the symbols at an engine */
  void registerSymbols(xabsl::Engine& engine);

  void update();
  void drawingOnField();


  // this stuff gets passed to xabsl
  // (free part angles)

  /** angle to the center of the largest free part of the opponent goal */
  float getCenterAngleOfFreePart();

  /** angle to the that side of the largest free part of the opponent goal nearest to the center of the opponent goal */
  float getInnerAngleOfFreePart();

  /** angle to the that side of the largest free part of the opponent goal farthest from the center of the opponent goal */
  float getOuterAngleOfFreePart();

  /** width of opening angle of largest free part of opponent goal */
  float getOpeningAngleOfFreePart();

  /** tolerance to angle to the center of the largest free part of opponent goal (how accurate the robot will be positioned) */
  float getAngleToleranceToFreePart();

  float getAngleToLastSeen();
  float getDistanceToLastSeen();

  Vector2<> innerSide;
  Vector2<> outerSide;


  // this stuff gets passed to xabsl
  // (other perceptions)
  float timeSinceOppGoalWasSeen;
  float timeSinceOwnGoalWasSeen;
  float timeSinceAnyGoalWasSeen;
  float goalsSeenInReadyState;
  float goalsSeenSinceLastPickup;
  bool  oppGoalWasSeen;
  bool  goalWasSeen;
};
