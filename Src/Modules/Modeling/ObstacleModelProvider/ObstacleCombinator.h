/**
* @file ObstacleCombinator.h
*
* This file declares a module that merges information from the ultrasonic obstacle grid
* and perceptions from vision.
*
* @author <a href="mailto:Tim.Laue@dfki.de">Tim Laue</a>
*/

#pragma once

#include "Representations/Modeling/ObstacleModel.h"
#include "Representations/Modeling/RobotsModel.h"
#include "Representations/Modeling/ArmContactModel.h"
#include "Representations/Modeling/USObstacleGrid.h"
#include "Representations/MotionControl/OdometryData.h"
#include "Tools/Module/Module.h"
#include <stack>


MODULE(ObstacleCombinator)
  REQUIRES(USObstacleGrid)
  REQUIRES(RobotsModel)
  REQUIRES(ArmContactModel)
  REQUIRES(OdometryData)
  PROVIDES_WITH_MODIFY_AND_DRAW(ObstacleModel)
END_MODULE


/**
* @class ObstacleCombinator
*
* A module for computing the occupied space in the robot's environment
*/
class ObstacleCombinator: public ObstacleCombinatorBase
{
private:
  /**
  * @class Parameters
  * The parameters of the module
  */
  class Parameters: public Streamable
  {
  private:
    /**
    * The method makes the object streamable.
    * @param in The stream from which the object is read
    * @param out The stream to which the object is written
    */
    virtual void serialize(In* in, Out* out)
    {
      STREAM_REGISTER_BEGIN();
      STREAM(maxRobotDistance);
      STREAM(considerStandingRobots);
      STREAM(considerLyingRobots);
      STREAM(considerArmCollisions);
      STREAM(minClusterSize);
      STREAM(clusterNonMaxThresholdCells);
      STREAM(covarianceFactor);
      STREAM_REGISTER_FINISH();
    }

  public:
    int maxRobotDistance;                /**< Maximum distance for considering robots as obstacles */
    bool considerStandingRobots;         /**< Use standing robots for obstacle modeling or not */
    bool considerLyingRobots;            /**< Use lying robots for obstacle modeling or not */
    bool considerArmCollisions;          /**< Use robots detected by arm collision for obstacle modeling or not */
    int minClusterSize;                  /**< Minimum size of cell cluster */
    bool clusterNonMaxThresholdCells;    /**< if true, a cluster might contain cells that did not have reached the threshold */
    float covarianceFactor;              /**< proportionality factor for the covariance > */

    /** Constructor */
    Parameters(): maxRobotDistance(1400), considerStandingRobots(false), considerLyingRobots(true), considerArmCollisions(true),
      minClusterSize(1), clusterNonMaxThresholdCells(false), covarianceFactor(10)
    {}
  };

  /**
  * @class CellCluster
  * A cluster of cells representing a single obstacle
  */
  class CellCluster
  {
  public:
    vector< Vector2<int> > cells;

    /** Constructor*/
    CellCluster()
    {
      cells.reserve(100);
    }

    /** Resets the cluster*/
    void init()
    {
      cells.clear();
    }
  };

public:
  /** Constructor */
  ObstacleCombinator();

private:
  Parameters parameters;                             /**< The parameters of this module */
  int clusterAssignment[USObstacleGrid::GRID_SIZE];  /**< Grid cluster information */
  CellCluster currentCluster;                        /**< The currently expanded cell cluster */
  stack< Vector2<int> > cellStack;                   /**< A stack for the floodfill algorithm */

  /** Executes this module
  * @param obstacleModel The data structure that is filled by this module
  */
  void update(ObstacleModel& obstacleModel);

  /** Computes model data from the grid
  * @param obstacles The model
  */
  void generateObstacleFromCurrentCluster(std::vector<ObstacleModel::Obstacle>& obstacles);

  /** Converts coordinates in grid coordinates to the local coordinate system
  * @param p The position within the grid
  * @return A position in local robot coordinates
  */
  Vector2<> gridToWorld(const Vector2<>& p) const;

  /** rotates the covariance by a given angle */
  static void rotateMatrix(Matrix2x2<>& matrix, const float angle);
};
