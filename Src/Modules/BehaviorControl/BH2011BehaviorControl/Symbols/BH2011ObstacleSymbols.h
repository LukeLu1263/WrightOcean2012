/**
* @file BH2011ObstacleSymbols.h
* Declaration of class BH2011ObstacleSymbols
* @author <a href="mailto:Tim.Laue@dfki.de">Tim Laue</a>
* @author Colin Graf
*/

#pragma once

#include "Modules/BehaviorControl/Symbols.h"
#include "Representations/Perception/RegionPercept.h"
#include "Representations/Modeling/ObstacleModel.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Modeling/ArmContactModel.h"
#include "Representations/Perception/BallPercept.h"
#include <list>
/**
* @class BH2011ObstacleSymbols
* Symbols for duelling
*/
class BH2011ObstacleSymbols : public Symbols
{
public:
  /** Constructor */
  BH2011ObstacleSymbols(const FrameInfo& frameInfo, const ObstacleModel& obstacleModel, const ArmContactModel& armContactModel,const RegionPercept& regionPercept, const BallPercept& ballPercept):
    frameInfo(frameInfo), obstacleModel(obstacleModel), armContactModel(armContactModel), lastTime(0) ,regionPercept(regionPercept), ballPercept(ballPercept){}

  /** Registers the symbols at an engine */
  void registerSymbols(xabsl::Engine& engine);

private:
  const FrameInfo& frameInfo;              /**< Current time */
  const ObstacleModel& obstacleModel;      /**< Reference to ObstacleModel which contains obstacle information */
  const ArmContactModel& armContactModel;  /**< Indication of something touching an arm  */
  const RegionPercept& regionPercept;
  const BallPercept& ballPercept;
  
  float distanceToClosestCenterLeft; /**< As the name says... */
  float distanceToClosestCenterRight; /**< As the name says... */
  float distanceToClosestCenter; /**< As the name says... */
  float distanceToClosest;
  float angleToClosest;
  float freeKickAngleLeft;
  float freeKickAngleRight;
  float leftRatio, rightRatio, leftRatioBall, rightRatioBall;  

  float leftRatioBallFiltered, rightRatioBallFiltered;  
  std::list<float> listLeftRatioBall, listRightRatioBall;

  unsigned int lastTime; /**< The time when \c distanceToClisestCenterLeft, \c distanceToClosestCenterRight and \c distanceToClosestCenter was computed */

  void computeDistanceToClosestCenterLeftAndRightAndCenterAndSomethingElse();
  float getDistanceToClosestCenterLeft();
  float getDistanceToClosestCenterRight();
  float getDistanceToClosestCenter();
  float getDistanceToClosest();
  float getAngleToClosest();
  float getFreeKickAngleLeft();
  float getFreeKickAngleRight();
  bool  getArmLeft();
  bool  getArmRight();
  
  void updateVisionObstacle();
  void updateVisionObstacleFiltered();
  void update();
};
