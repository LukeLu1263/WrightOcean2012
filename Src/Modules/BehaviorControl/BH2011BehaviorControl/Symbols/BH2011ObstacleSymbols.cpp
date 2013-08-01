/**
* @file BH2011ObstacleSymbols.cpp
* Implementation of class BH2011ObstacleSymbols.
* @author <a href="mailto:Tim.Laue@dfki.de">Tim Laue</a>
* @author Colin Graf
*/

#include "BH2011ObstacleSymbols.h"

void BH2011ObstacleSymbols::registerSymbols(xabsl::Engine& engine)
{
  engine.registerDecimalInputSymbol("obstacle.distance_to_closest_center_left", this, &BH2011ObstacleSymbols::getDistanceToClosestCenterLeft);
  engine.registerDecimalInputSymbol("obstacle.distance_to_closest_center_right", this, &BH2011ObstacleSymbols::getDistanceToClosestCenterRight);
  engine.registerDecimalInputSymbol("obstacle.distance_to_closest_center", this, &BH2011ObstacleSymbols::getDistanceToClosestCenter);
  engine.registerDecimalInputSymbol("obstacle.distance_to_closest", this, &BH2011ObstacleSymbols::getDistanceToClosest);
  engine.registerDecimalInputSymbol("obstacle.angle_to_closest", this, &BH2011ObstacleSymbols::getAngleToClosest);
  engine.registerDecimalInputSymbol("obstacle.free_kick_angle_left", this, &BH2011ObstacleSymbols::getFreeKickAngleLeft);
  engine.registerDecimalInputSymbol("obstacle.free_kick_angle_right", this, &BH2011ObstacleSymbols::getFreeKickAngleRight);
  engine.registerBooleanInputSymbol("obstacle.arm_left", this, &BH2011ObstacleSymbols::getArmLeft);
  engine.registerBooleanInputSymbol("obstacle.arm_right", this, &BH2011ObstacleSymbols::getArmRight);
  
  // Vision Based Obstacle Avoidance
  engine.registerDecimalInputSymbol("obstacle.vision.leftRatio", &leftRatio);
  engine.registerDecimalInputSymbol("obstacle.vision.rightRatio", &rightRatio);
  engine.registerDecimalInputSymbol("obstacle.vision.leftRatioBall", &leftRatioBall);
  engine.registerDecimalInputSymbol("obstacle.vision.rightRatioBall", &rightRatioBall);
  engine.registerDecimalInputSymbol("obstacle.vision.leftRatioBallFiltered", &leftRatioBallFiltered);
  engine.registerDecimalInputSymbol("obstacle.vision.rightRatioBallFiltered", &rightRatioBallFiltered);
}

void BH2011ObstacleSymbols::update()
{
	 updateVisionObstacle(); 
}

void BH2011ObstacleSymbols::updateVisionObstacle()
{  
	if (regionPercept.visionObstacleValid) {
		int countLeft = 1, countRight = 1, countFeasibleLeft = 0, countFeasibleRight = 0;
		for (unsigned int i = 0; i < regionPercept.vecCountFeasibleLeft.size(); i++) {
			countLeft += regionPercept.vecCountLeft[i];
			countRight += regionPercept.vecCountRight[i];
			countFeasibleLeft += regionPercept.vecCountFeasibleLeft[i];
			countFeasibleRight += regionPercept.vecCountFeasibleRight[i];
		}
		leftRatio = (float)countFeasibleLeft / countLeft;
		rightRatio = (float)countFeasibleRight / countRight;

		if (ballPercept.ballWasSeen && ballPercept.positionInImage.y > 65) {
			int countLeftBall = 1, countRightBall = 1, countFeasibleLeftBall = 0, countFeasibleRightBall = 0;
			for (unsigned int i = (unsigned int)((ballPercept.positionInImage.y-72)/regionPercept.gridStepSize); i < regionPercept.vecCountFeasibleLeft.size(); i++) {
				countLeftBall += regionPercept.vecCountLeft[i];
				countRightBall += regionPercept.vecCountRight[i];
				countFeasibleLeftBall += regionPercept.vecCountFeasibleLeft[i];
				countFeasibleRightBall += regionPercept.vecCountFeasibleRight[i];
			}
			leftRatioBall = (float)countFeasibleLeftBall / countLeftBall;
			rightRatioBall = (float)countFeasibleRightBall / countRightBall;

			listLeftRatioBall.push_back(leftRatioBall);
			listRightRatioBall.push_back(rightRatioBall);

			if (listLeftRatioBall.size() >= 4) {
				listLeftRatioBall.pop_front();
			}
			if (listRightRatioBall.size() >= 4) {
				listRightRatioBall.pop_front();
			}
									
		} else {
			leftRatioBall = 0.0;
			rightRatioBall = 0.0;		

			if (!listLeftRatioBall.empty()) {
				listLeftRatioBall.pop_front();
			}
			if (!listRightRatioBall.empty()) {
				listRightRatioBall.pop_front();
			}
		}
	} else {
		/*
		leftRatio = 1.0;
		rightRatio = 1.0;
		leftRatioBall = 1.0;
		rightRatioBall = 1.0;		

		if (!listLeftRatioBall.empty()) {
				listLeftRatioBall.pop_front();
			}
		if (!listRightRatioBall.empty()) {
			listRightRatioBall.pop_front();
		}
		*/
	}
	
	updateVisionObstacleFiltered();

	//fprintf(stderr, "%f %f, B: %f %f, BF: %f %f\n", leftRatio, rightRatio, leftRatioBall, rightRatioBall, leftRatioBallFiltered, rightRatioBallFiltered);
}

void BH2011ObstacleSymbols::updateVisionObstacleFiltered()
{
	//printf("L [%f] ", leftRatioBall);
	if (listLeftRatioBall.size() != 0) {
		leftRatioBallFiltered = 0.0;
	
		for (std::list<float>::iterator iter = listLeftRatioBall.begin(); iter != listLeftRatioBall.end(); iter++) {
			leftRatioBallFiltered += *iter;

			//printf("%f", *iter);
		}
		leftRatioBallFiltered /= listLeftRatioBall.size();
	} else {
		leftRatioBallFiltered = 0.0;
	}
	//printf("<%f> ", leftRatioBallFiltered);

	//printf("R [%f] ", rightRatioBall);
	if (listRightRatioBall.size() != 0) {
		rightRatioBallFiltered = 0.0;
	
		for (std::list<float>::iterator iter = listRightRatioBall.begin(); iter != listRightRatioBall.end(); iter++) {
			rightRatioBallFiltered += *iter;

			//printf("%f ", *iter);
		}
		rightRatioBallFiltered /= listRightRatioBall.size();
	} else {
		rightRatioBallFiltered = 0.0;
	}
	//printf("<%f>\n", rightRatioBallFiltered);
}


void BH2011ObstacleSymbols::computeDistanceToClosestCenterLeftAndRightAndCenterAndSomethingElse()
{
  if(lastTime == frameInfo.time)
    return;
  lastTime = frameInfo.time;
  freeKickAngleLeft = 0.f;
  freeKickAngleRight = 0.f;
  const float maxDistance = 3000.f; // infinite
  const float maxSqrDistance = maxDistance * maxDistance;
  const float maxKickAngleDistance = 1000.f;
  const float maxSqrKickAngleDistance = maxKickAngleDistance * maxKickAngleDistance;
  const float minKickOpeningAngle = fromDegrees(15.f) / 2.f;
  distanceToClosest = distanceToClosestCenterLeft = distanceToClosestCenterRight = maxSqrDistance;
  angleToClosest = 0.f;

  for(std::vector<ObstacleModel::Obstacle>::const_iterator iter = obstacleModel.obstacles.begin(), end = obstacleModel.obstacles.end(); iter != end; ++iter)
  {
    const ObstacleModel::Obstacle& obstacle = *iter;

    // Compute distance and angle to closest obstacle:
    float sqrAbs = obstacle.closestPoint.squareAbs();
    if(sqrAbs < distanceToClosest)
    {
      distanceToClosest = sqrAbs;
      angleToClosest = obstacle.closestPoint.angle();
    }

    // Compute distances to sectors:
    if(sqrAbs < distanceToClosestCenterLeft)
      if((obstacle.rightCorner.x > 0.f && obstacle.rightCorner.y >= 0.f && obstacle.rightCorner.y < 150.f) || // right corner point in left rectangle
          (obstacle.leftCorner.x > 0.f && obstacle.leftCorner.y >= 0.f && obstacle.leftCorner.y < 150.f) ||  // left corner point in left rectangle
          ((obstacle.rightCorner.x > 0.f || obstacle.leftCorner.x > 0.f) && obstacle.rightCorner.y < 0.f && obstacle.leftCorner.y > 0.f)) // line segement from left corner to right corner crosses the left rectangle
        distanceToClosestCenterLeft = sqrAbs;
    if(sqrAbs < distanceToClosestCenterRight)
      if((obstacle.rightCorner.x > 0.f && obstacle.rightCorner.y > -150.f && obstacle.rightCorner.y <= 0.f) || // right corner point in right rectangle
          (obstacle.leftCorner.x > 0.f && obstacle.leftCorner.y > -150.f && obstacle.leftCorner.y <= 0.f) ||  // left corner point in right rectangle
          ((obstacle.rightCorner.x > 0.f || obstacle.leftCorner.x > 0.f) && obstacle.rightCorner.y < 0.f && obstacle.leftCorner.y > 0.f)) // line segement from left corner to right corner crosses the right rectangle
        distanceToClosestCenterRight = sqrAbs;

    // Compute kick angles:
    if(sqrAbs < maxSqrKickAngleDistance) // Consider this obstacle
    {
      float leftObstacleAngle = obstacle.leftCorner.angle();
      float rightObstacleAngle = obstacle.rightCorner.angle();
      // Check, if right angle is inside obstacle:
      if((leftObstacleAngle + minKickOpeningAngle > freeKickAngleLeft) && (rightObstacleAngle  - minKickOpeningAngle < freeKickAngleLeft))
      {
        freeKickAngleLeft = leftObstacleAngle + minKickOpeningAngle;
      }
      // Check, if left angle is inside obstacle:
      if((leftObstacleAngle  + minKickOpeningAngle > freeKickAngleRight) && (rightObstacleAngle - minKickOpeningAngle < freeKickAngleRight))
      {
        freeKickAngleRight = rightObstacleAngle - minKickOpeningAngle;
      }
    }
  }

  distanceToClosest = distanceToClosest < maxSqrDistance ? sqrt(distanceToClosest) : maxDistance;
  distanceToClosestCenterLeft = distanceToClosestCenterLeft < maxSqrDistance ? sqrt(distanceToClosestCenterLeft) : maxDistance;
  distanceToClosestCenterRight = distanceToClosestCenterRight < maxSqrDistance ? sqrt(distanceToClosestCenterRight) : maxDistance;
  distanceToClosestCenter = distanceToClosestCenterLeft < distanceToClosestCenterRight ? distanceToClosestCenterLeft : distanceToClosestCenterRight;
  angleToClosest = toDegrees(angleToClosest);
  freeKickAngleLeft = toDegrees(freeKickAngleLeft);
  freeKickAngleRight = toDegrees(freeKickAngleRight);
}

float BH2011ObstacleSymbols::getDistanceToClosestCenterLeft()
{
  computeDistanceToClosestCenterLeftAndRightAndCenterAndSomethingElse();
  return distanceToClosestCenterLeft;
}

float BH2011ObstacleSymbols::getDistanceToClosestCenterRight()
{
  computeDistanceToClosestCenterLeftAndRightAndCenterAndSomethingElse();
  return distanceToClosestCenterRight;
}

float BH2011ObstacleSymbols::getDistanceToClosestCenter()
{
  computeDistanceToClosestCenterLeftAndRightAndCenterAndSomethingElse();
  return distanceToClosestCenter;
}

float BH2011ObstacleSymbols::getDistanceToClosest()
{
  computeDistanceToClosestCenterLeftAndRightAndCenterAndSomethingElse();
  return distanceToClosest;
}

float BH2011ObstacleSymbols::getAngleToClosest()
{
  computeDistanceToClosestCenterLeftAndRightAndCenterAndSomethingElse();
  return angleToClosest;
}

float BH2011ObstacleSymbols::getFreeKickAngleLeft()
{
  computeDistanceToClosestCenterLeftAndRightAndCenterAndSomethingElse();
  return freeKickAngleLeft;
}

float BH2011ObstacleSymbols::getFreeKickAngleRight()
{
  computeDistanceToClosestCenterLeftAndRightAndCenterAndSomethingElse();
  return freeKickAngleRight;
}

bool BH2011ObstacleSymbols::getArmLeft()
{
  return armContactModel.contactLeft;
}

bool BH2011ObstacleSymbols::getArmRight()
{
  return armContactModel.contactRight;
}
