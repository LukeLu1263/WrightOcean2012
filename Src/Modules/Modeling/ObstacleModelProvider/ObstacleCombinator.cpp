/**
* @file ObstacleCombinator.cpp
*
* This file implements a module that merges information from the ultrasonic obstacle grid
* and perceptions from vision.
*
* @author <a href="mailto:Tim.Laue@dfki.de">Tim Laue</a>
*/

#include "ObstacleCombinator.h"
#include "Tools/Streams/InStreams.h"
#include "Tools/Settings.h"
#include "Tools/Math/Vector3.h"
#include "Tools/Math/Geometry.h"
#include "Tools/Team.h"
#include <cfloat>


ObstacleCombinator::ObstacleCombinator()
{
  InConfigMap stream(Global::getSettings().expandLocationFilename("obstacleCombinator.cfg"));
  if(stream.exists())
    stream >> parameters;
}

void ObstacleCombinator::update(ObstacleModel& obstacleModel)
{
  MODIFY("parameters:ObstacleCombinator", parameters);

  // Perform grid cell clustering and add clusters as obstacles:
  memset(clusterAssignment, 0, USObstacleGrid::GRID_SIZE * sizeof(int));
  obstacleModel.obstacles.clear();
  currentCluster.init();
  int currentClusterIndex(1);
  for(int y = 1; y < USObstacleGrid::GRID_LENGTH - 1; ++y)
  {
    for(int x = 1; x < USObstacleGrid::GRID_LENGTH - 1; ++x)
    {
      const int currentCellIndex = y * USObstacleGrid::GRID_LENGTH + x;
      const USObstacleGrid::Cell& c = theUSObstacleGrid.cells[currentCellIndex];
      if(clusterAssignment[currentCellIndex] == 0 && c.state >= theUSObstacleGrid.cellOccupiedThreshold)
      {
        clusterAssignment[currentCellIndex] = currentClusterIndex;
        // iterative implementation of floodfill algorithm
        cellStack.push(Vector2<int>(x, y));
        while(!cellStack.empty())
        {
          int x = cellStack.top().x;
          int y = cellStack.top().y;
          cellStack.pop();
          currentCluster.cells.push_back(Vector2<int>(x, y));
          if(x == 0 || y == 0 || x == USObstacleGrid::GRID_LENGTH - 1 || y == USObstacleGrid::GRID_LENGTH - 1) //ignore border
            continue;
          // Test all eight neighbors (also stupid check for center cell)
          for(int y2 = y - 1; y2 <= y + 1; ++y2)
          {
            for(int x2 = x - 1; x2 <= x + 1; ++x2)
            {
              const int neighborCellIndex = y2 * USObstacleGrid::GRID_LENGTH + x2;
              const USObstacleGrid::Cell& cn = theUSObstacleGrid.cells[neighborCellIndex];
              if(clusterAssignment[neighborCellIndex] == 0 &&
                 ((cn.state >= theUSObstacleGrid.cellOccupiedThreshold && !parameters.clusterNonMaxThresholdCells) ||
                  (cn.state > 0 && parameters.clusterNonMaxThresholdCells)))
              {
                clusterAssignment[neighborCellIndex] = currentClusterIndex;
                cellStack.push(Vector2<int>(x2, y2));
              }
            }
          }
        }
        ++currentClusterIndex;
        if(currentCluster.cells.size() >= (unsigned int)(parameters.minClusterSize))
          generateObstacleFromCurrentCluster(obstacleModel.obstacles);
        currentCluster.init();
      }
    }
  }

  // Robot dimensions:
  const float robotHeight = 580;
  const float robotWidth  = 300; // Guessed value
  const float robotDepth  = 150; // Guessed value
  const int   robotSize   = 9;   // Guessed value

  // Add robot obstacles:
  for(RobotsModel::RCIt robot = theRobotsModel.robots.begin(); robot != theRobotsModel.robots.end(); robot++)
  {
    float robotDistance = robot->relPosOnField.abs();
    if((!robot->standing && parameters.considerLyingRobots) && (robotDistance < parameters.maxRobotDistance))
    {
      Vector2<> widthOffset(robotHeight / 2.0f, 0.0f);
      Vector2<> robotCenter(robot->relPosOnField);
      Vector2<> closestRobotPoint(robotCenter);
      closestRobotPoint.normalize(robotCenter.abs() - robotWidth / 2.0f); // Assume that robot lies crosswise
      Vector2<> leftCorner(widthOffset);
      leftCorner.rotateLeft();
      leftCorner += robotCenter;
      Vector2<> rightCorner(widthOffset);
      rightCorner.rotateRight();
      rightCorner += robotCenter;
      obstacleModel.obstacles.push_back(ObstacleModel::Obstacle(leftCorner, rightCorner, robotCenter, closestRobotPoint,
                                        robotSize, robot->covariance, ObstacleModel::Obstacle::ROBOT));
    }
    else if((robot->standing && parameters.considerStandingRobots) && (robotDistance < parameters.maxRobotDistance))
    {
      Vector2<> widthOffset(robotWidth / 2.0f, 0.0f);
      Vector2<> robotCenter(robot->relPosOnField);
      Vector2<> closestRobotPoint(robotCenter);
      closestRobotPoint.normalize(robotCenter.abs() - robotWidth / 2.0f);
      Vector2<> leftCorner(widthOffset);
      leftCorner.rotateLeft();
      leftCorner += robotCenter;
      Vector2<> rightCorner(widthOffset);
      rightCorner.rotateRight();
      rightCorner += robotCenter;
      obstacleModel.obstacles.push_back(ObstacleModel::Obstacle(leftCorner, rightCorner, robotCenter, closestRobotPoint,
                                        robotSize, robot->covariance, ObstacleModel::Obstacle::ROBOT));
    }
  }

  // Add arm obstacles:
  Vector2<> robotCenter(0.0f, robotWidth / 2.0f + robotDepth / 2.0f);
  float robotDistance = robotCenter.abs();
  if(parameters.considerArmCollisions
     && (theArmContactModel.contactLeft || theArmContactModel.contactRight)
     && (robotDistance < parameters.maxRobotDistance))
  {
    // Assume chest to arm contact
    Vector2<> widthOffset(0.0f, robotWidth / 2.0f);
    Vector2<> closestRobotPoint(0.0f, robotWidth / 2.0f);
    Vector2<> leftCorner(widthOffset);
    leftCorner.rotateRight();
    leftCorner += robotCenter;
    Vector2<> rightCorner(widthOffset);
    rightCorner.rotateLeft();
    rightCorner += robotCenter;
    Matrix2x2<> covariance(robotWidth / 2.0f, 0.0f, 0.0f, robotWidth / 2.0f);
    if(theArmContactModel.contactLeft)
      obstacleModel.obstacles.push_back(ObstacleModel::Obstacle(leftCorner, rightCorner, robotCenter, closestRobotPoint,
                                        robotSize, covariance, ObstacleModel::Obstacle::ARM));
    if(theArmContactModel.contactRight)
      obstacleModel.obstacles.push_back(ObstacleModel::Obstacle(-leftCorner, -rightCorner, -robotCenter, -closestRobotPoint,
                                        robotSize, covariance, ObstacleModel::Obstacle::ARM));
  }

  TEAM_OUTPUT(idTeamMateObstacleModel, bin, obstacleModel);
}

void ObstacleCombinator::generateObstacleFromCurrentCluster(std::vector<ObstacleModel::Obstacle>& obstacles)
{
  Vector2<int>& c = currentCluster.cells[0];
  int xMin(c.x);
  int yMin(c.y);
  int xMax(c.x);
  int yMax(c.y);
  float rightAngle = 10.0;
  float leftAngle = -10.0;
  Vector2<int> rightCorner;
  Vector2<int> leftCorner;
  Vector2<> centerCells;
  Vector2<int> robotPosition(USObstacleGrid::GRID_LENGTH / 2, USObstacleGrid::GRID_LENGTH / 2);
  Vector2<int> closestPoint(USObstacleGrid::GRID_LENGTH, USObstacleGrid::GRID_LENGTH);
  int closestPointSqrDist(USObstacleGrid::GRID_SIZE * 2);
  for(unsigned int i = 0; i < currentCluster.cells.size(); ++i)
  {
    c = currentCluster.cells[i];
    centerCells.x += c.x;
    centerCells.y += c.y;
    Vector2<int> point(c.x - robotPosition.x, c.y - robotPosition.y);
    int sqrDistToRobot = sqr(point.x) + sqr(point.y);
    float angleToRobot = atan2(static_cast<float>(point.y), static_cast<float>(point.x));
    if(angleToRobot < rightAngle)
    {
      rightAngle = angleToRobot;
      rightCorner = Vector2<int>(c.x, c.y);
    }
    if(angleToRobot > leftAngle)
    {
      leftAngle = angleToRobot;
      leftCorner = Vector2<int>(c.x, c.y);
    }
    if(sqrDistToRobot < closestPointSqrDist)
    {
      closestPoint = c;
      closestPointSqrDist = sqrDistToRobot;
    }
    if(c.x < xMin)
      xMin = c.x;
    else if(c.x > xMax)
      xMax = c.x;
    if(c.y < yMin)
      yMin = c.y;
    else if(c.y > yMax)
      yMax = c.y;
  }
  centerCells /= static_cast<float>(currentCluster.cells.size());

  const float angleToCenterPoint = Geometry::angleTo(Pose2D(USObstacleGrid::GRID_LENGTH / 2, USObstacleGrid::GRID_LENGTH / 2), centerCells); //calculates the angle to the center of the cluster in grid coordinate system (independent of robot rotation)
  const float cosinus = cos(-angleToCenterPoint);
  const float sinus = sin(-angleToCenterPoint);
  float newX;
  float newY;

  //initializing of the rectangle
  float xMinRotated(FLT_MAX);
  float yMinRotated(FLT_MAX);
  float xMaxRotated(-FLT_MAX);
  float yMaxRotated(-FLT_MAX);

  for(unsigned int i = 0; i < currentCluster.cells.size(); ++i)
  {
    newX = cosinus * currentCluster.cells[i].x - sinus * currentCluster.cells[i].y; // rotates each cell of the cluster
    newY = sinus * currentCluster.cells[i].x + cosinus * currentCluster.cells[i].y;
    //sets new values for rectangle
    if(newX < xMinRotated)
      xMinRotated = newX;
    else if(newX > xMaxRotated)
      xMaxRotated = newX;
    if(newY < yMinRotated)
      yMinRotated = newY;
    else if(newY > yMaxRotated)
      yMaxRotated = newY;
  }

  Vector2<> closestPointWorld = gridToWorld(Vector2<>(closestPoint.x + 0.5f, closestPoint.y + 0.5f));
  Vector2<> centerWorld = gridToWorld(centerCells);

  //expansion (length of x- and y-axis through the center point) and orientation (dependent on robot rotation) of the cluster
  Vector3<> covarianceEllipse(((xMaxRotated - xMinRotated) * USObstacleGrid::CELL_SIZE) * parameters.covarianceFactor, ((yMaxRotated - yMinRotated) * USObstacleGrid::CELL_SIZE) * parameters.covarianceFactor, atan2(centerWorld.y, centerWorld.x));
  Matrix2x2<> covariance(covarianceEllipse.x, 0, 0, covarianceEllipse.y); // covariance is initialised with uncorrelated values (only expansion of cluster as variances)
  rotateMatrix(covariance, covarianceEllipse.z); // rotates the covariance so that it fits to the orientation and expansion of the cluster

  obstacles.push_back(ObstacleModel::Obstacle(gridToWorld(Vector2<>((float)(leftCorner.x), (float)(leftCorner.y))),
                      gridToWorld(Vector2<>((float)(rightCorner.x), (float)(rightCorner.y))), centerWorld, closestPointWorld,
                      static_cast<int>(currentCluster.cells.size()), covariance));
}

inline Vector2<> ObstacleCombinator::gridToWorld(const Vector2<>& p) const
{
  Vector2<> pWorld(p);
  const float move(USObstacleGrid::GRID_LENGTH / 2);
  pWorld -= Vector2<>(move, move);
  pWorld *= USObstacleGrid::CELL_SIZE;
  Pose2D odoRotation(-theOdometryData.rotation, 0.0f, 0.0f);
  return odoRotation * pWorld;
}

void ObstacleCombinator::rotateMatrix(Matrix2x2<>& matrix, const float angle)
{
  const float cosine = cos(angle);
  const float sine = sin(angle);
  const Matrix2x2<> rotationMatrix(cosine, -sine, sine, cosine);
  matrix = (rotationMatrix * matrix) * rotationMatrix.transpose();
}

MAKE_MODULE(ObstacleCombinator, Modeling)
