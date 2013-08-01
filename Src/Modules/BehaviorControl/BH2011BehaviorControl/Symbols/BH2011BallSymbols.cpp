/**
* @file BH2011BallSymbols.cpp
*
* Implementation of class BallSymbols.
*
* @author Max Risler
* \author Colin Graf
*/

#include "BH2011BallSymbols.h"
#include "Tools/Debugging/Modify.h"
#include "Modules/Infrastructure/TeamDataProvider.h"

void BH2011BallSymbols::registerSymbols(xabsl::Engine& engine)
{
  engine.registerDecimalInputSymbol("ball.position.field.x", &ballPositionField.x);
  engine.registerDecimalInputSymbol("ball.position.field.y", &ballPositionField.y);
  engine.registerDecimalInputSymbol("ball.end_position.field.x", &ballEndPositionField.x);
  engine.registerDecimalInputSymbol("ball.end_position.field.y", &ballEndPositionField.y);

  engine.registerDecimalInputSymbol("ball.x", &ballPositionRel.x);
  engine.registerDecimalInputSymbol("ball.y", &ballPositionRel.y);
  engine.registerDecimalInputSymbol("ball.kick.x", &ballKickRel.x);
  engine.registerDecimalInputSymbol("ball.kick.y", &ballKickRel.y);
  engine.registerDecimalInputSymbol("ball.end_position.x", &ballEndPositionRel.x);
  engine.registerDecimalInputSymbol("ball.end_position.y", &ballEndPositionRel.y);
  //This is a little bit faster with the Position because it's just using the last percept
  engine.registerDecimalInputSymbol("ball.seen.x", &seenBallPositionRel.x);
  engine.registerDecimalInputSymbol("ball.seen.y", &seenBallPositionRel.y);
  engine.registerDecimalInputSymbol("ball.seen.estimate.x", this, &BH2011BallSymbols::getBallLastSeenEstimateX);
  engine.registerDecimalInputSymbol("ball.seen.estimate.y", this, &BH2011BallSymbols::getBallLastSeenEstimateY);
  engine.registerDecimalInputSymbol("ball.position.seen.estimate.x", this, &BH2011BallSymbols::getBallPositionLastSeenEstimateX);
  engine.registerDecimalInputSymbol("ball.position.seen.estimate.y", this, &BH2011BallSymbols::getBallPositionLastSeenEstimateY);
  engine.registerDecimalInputSymbol("ball.seen.angle", this, &BH2011BallSymbols::getSeenBallAngle);
  engine.registerDecimalInputSymbol("ball.seen.distance", this, &BH2011BallSymbols::getBallSeenDistance);

  engine.registerDecimalInputSymbol("ball.distance", this, &BH2011BallSymbols::getBallDistance);
  engine.registerDecimalInputSymbol("ball.angle", this, &BH2011BallSymbols::getBallAngle);
  engine.registerBooleanInputSymbol("ball.was_seen", &ballWasSeen);
  engine.registerDecimalInputSymbol("ball.time_since_last_seen", &timeSinceBallWasSeen);
  engine.registerDecimalInputSymbol("ball.time_since_disappeared", this, &BH2011BallSymbols::getTimeSinceDisappeared);

  engine.registerDecimalInputSymbol("ball.speed.field.x", &ballSpeedField.x);
  engine.registerDecimalInputSymbol("ball.speed.field.y", &ballSpeedField.y);
  engine.registerDecimalInputSymbol("ball.speed.robot.x", &ballSpeedRel.x);
  engine.registerDecimalInputSymbol("ball.speed.robot.y", &ballSpeedRel.y);

  engine.registerDecimalInputSymbol("ball.time_when_own_y_axis_reached", this, &BH2011BallSymbols::getTimeWhenBallReachesOwnYAxis);
  engine.registerDecimalInputSymbol("ball.position_when_ball_reaches_own_y_axis.y", this, &BH2011BallSymbols::getYPosWhenBallReachesOwnYAxis);

  engine.registerDecimalInputSymbol("ball.distance.own_goal", this, &BH2011BallSymbols::getBallDistanceToOwnGoal);

  engine.registerDecimalInputSymbol("ball.on_field.x", this, &BH2011BallSymbols::getOnFieldX);
  engine.registerDecimalInputSymbol("ball.on_field.y", this, &BH2011BallSymbols::getOnFieldY);
  engine.registerDecimalInputSymbol("ball.on_field.angle", this, &BH2011BallSymbols::getOnFieldAngle);
  engine.registerDecimalInputSymbol("ball.on_field.speed.x", this, &BH2011BallSymbols::getOnFieldSpeedX);
  engine.registerDecimalInputSymbol("ball.on_field.speed.y", this, &BH2011BallSymbols::getOnFieldSpeedY);
  
 // engine.registerBooleanInputSymbol("ball.isNearestToBall", &staticGetIsNearestToBall);
  engine.registerDecimalInputSymbol("ball.strikerPn", &strikerPn);
  engine.registerDecimalInputSymbol("ball.offensiveSupporterPn", &offensiveSupporterPn);
  engine.registerDecimalInputSymbol("ball.defensiveSupporterPn", &defensiveSupporterPn);

 // engine.registerBooleanInputSymbol("ball.isSeenTeam", &ballWasSeenTeam);
 // engine.registerDecimalInputSymbol("ball.team.x", &ballPositionTeamField.x);
 // engine.registerDecimalInputSymbol("ball.team.y", &ballPositionTeamField.y);

  engine.registerBooleanInputSymbol("ball.someone_is_kicking", &someoneIsKicking); 

// engine.registerDecimalInputSymbol("ball.num_conn_player", &getNumConnPlayer);
}


void BH2011BallSymbols::update()
{
  timeSinceBallWasSeen = (float) frameInfo.getTimeSince(ballModel.timeWhenLastSeen);
  ballWasSeen = timeSinceBallWasSeen < 500;
  const BallState& estimate = ballModel.estimate;
  ballPositionRel = estimate.position;
  ballEndPositionRel = ballModel.endPosition;
  ballEndPositionField = Geometry::relative2FieldCoord(robotPose, ballEndPositionRel);
  ballPositionField = estimate.getPositionInFieldCoordinates(robotPose);
  ballSpeedRel = estimate.velocity;
  ballSpeedField = estimate.getVelocityInFieldCoordinates(robotPose);
  seenBallPositionRel = ballModel.lastPerception.position;
  ballLastSeenEstimate = ballModel.lastSeenEstimate.position;
  ballPositionLastSeenEstimate = ballModel.lastSeenEstimate.getPositionInFieldCoordinates(robotPose);
  ballKickRel = calculateBallInRobotOrigin(estimate.position);

   //

  updateRole();
 
}


float BH2011BallSymbols::getBallLastSeenEstimateX()
{
  return ballLastSeenEstimate.x;
}

float BH2011BallSymbols::getBallLastSeenEstimateY()
{
  return ballLastSeenEstimate.y;
}

float BH2011BallSymbols::getBallPositionLastSeenEstimateX()
{
  return ballPositionLastSeenEstimate.x;
}

float BH2011BallSymbols::getBallPositionLastSeenEstimateY()
{
  return ballPositionLastSeenEstimate.y;
}


float BH2011BallSymbols::getBallFieldRobotX()
{
  return ballPositionField.x;
}

float BH2011BallSymbols::getBallFieldRobotY()
{
  return ballPositionField.y;
}

float BH2011BallSymbols::getBallPositionRobotX()
{
  return ballPositionRel.x;
}

float BH2011BallSymbols::getBallPositionRobotY()
{
  return ballPositionRel.y;
}

float BH2011BallSymbols::getBallAngle()
{
  return toDegrees(ballModel.estimate.getAngle());
}

float BH2011BallSymbols::getSeenBallAngle()
{
  return toDegrees(ballModel.lastPerception.getAngle());
}

float BH2011BallSymbols::getBallDistance()
{
  return ballModel.estimate.getDistance();
}

float BH2011BallSymbols::getBallSeenDistance()
{
  return ballModel.lastPerception.getDistance();
}

float BH2011BallSymbols::getTimeWhenBallReachesOwnYAxis()
{
  float result = 1000000;

  float decel = (float)fieldDimensions.ballFriction;
  if(ballModel.estimate.velocity.x == 0.0f)
    return result;
  float decelX = ballModel.estimate.velocity.x * decel / ballModel.estimate.velocity.abs();
  float decelTime = ballModel.estimate.velocity.x / decelX;
  float xPosWhenStopping = ballModel.estimate.position.x + decelTime * ballModel.estimate.velocity.x - 0.5f * decel * sqr(decelTime);
  if(xPosWhenStopping * ballModel.estimate.position.x < 0)
  {
    float p(ballModel.estimate.velocity.x * 2 / decelX), q(2.0f / decelX * ballModel.estimate.position.x);
    float temp =  p * -0.5f + sqrt(sqr(p * 0.5f) - q);
    return temp;
  }
  return result;
}

float BH2011BallSymbols::getYPosWhenBallReachesOwnYAxis()
{
  if(ballSpeedRel.x == 0 || ballPositionRel.x * ballSpeedRel.x > 0) // Ball does not move or moves away
  {
    return 0.0f;
  }
  float timeWhenAxisIsReached = abs(ballPositionRel.x / ballSpeedRel.x);
  Vector2<> finalBallPos = ballPositionRel + (ballSpeedRel * timeWhenAxisIsReached);

  MODIFY("behavior symbols:ball:finalBallPos", finalBallPos);

  return finalBallPos.y;
}

float BH2011BallSymbols::getBallDistanceToOwnGoal()
{
  return (ballPositionField - Vector2<>((float) fieldDimensions.xPosOwnGroundline, 0.f)).abs();
}

float BH2011BallSymbols::getTimeSinceDisappeared()
{
  return float(frameInfo.getTimeSince(ballHypotheses.timeWhenDisappeared));
}

float BH2011BallSymbols::getOnFieldX()
{
  return ballModel.estimate.getPositionInFieldCoordinates(robotPose).x;
}

float BH2011BallSymbols::getOnFieldY()
{
  return ballModel.estimate.getPositionInFieldCoordinates(robotPose).y;
}

float BH2011BallSymbols::getOnFieldAngle()
{
  Vector2<> ballOnField = ballModel.estimate.getPositionInFieldCoordinates(robotPose);
  return toDegrees((ballOnField - robotPose.translation).angle());
}

float BH2011BallSymbols::getOnFieldSpeedX()
{
  return ballModel.estimate.getVelocityInFieldCoordinates(robotPose).x;
}

float BH2011BallSymbols::getOnFieldSpeedY()
{
  return ballModel.estimate.getVelocityInFieldCoordinates(robotPose).y;
}

Vector2<> BH2011BallSymbols::calculateBallInRobotOrigin(const Vector2<>& ballRel)
{
  // calculate "center of hip" position from left foot
  Pose3D fromLeftFoot(torsoMatrix.rotation);
  fromLeftFoot.conc(robotModel.limbs[MassCalibration::footLeft]);
  fromLeftFoot.translate(0, 0, -robotDimensions.heightLeg5Joint);
  //fromLeftFoot.translation *= -1.;
  fromLeftFoot.rotation = torsoMatrix.rotation;

  // calculate "center of hip" position from right foot
  Pose3D fromRightFoot(torsoMatrix.rotation);
  fromRightFoot.conc(robotModel.limbs[MassCalibration::footRight]);
  fromRightFoot.translate(0, 0, -robotDimensions.heightLeg5Joint);
// fromRightFoot.translation *= -1.;
  fromRightFoot.rotation = torsoMatrix.rotation;

  // determine used foot
  const bool useLeft = fromLeftFoot.translation.z < fromRightFoot.translation.z;

  // calculate foot span
  const Vector3<> newFootSpan(fromRightFoot.translation - fromLeftFoot.translation);

  // and construct the matrix
  Pose3D newTorsoMatrix;
  newTorsoMatrix.translate(newFootSpan.x / (useLeft ? 2.f : -2.f), newFootSpan.y / (useLeft ? 2.f : -2.f), 0);
  //newTorsoMatrix.conc(useLeft ? fromLeftFoot : fromRightFoot);

  const Vector3<> foot(useLeft ? fromLeftFoot.translation : fromRightFoot.translation);

  return Vector2<>(ballRel.x + newTorsoMatrix.translation.x + foot.x, ballRel.y + newTorsoMatrix.translation.y + foot.y);
}




void BH2011BallSymbols::updateStrikerPn()
{	 
	for (int p = 1; p < TeamMateData::numOfPlayers; p++) {
		isPlayerValid[p] = false;
	}

	// Activate self & connected teammates
	isPlayerValid[robotInfo.number] = true;		
	if (teamMateData.numOfConnectedPlayers >= 1)
		isPlayerValid[teamMateData.firstTeamMate] = true;	
	
	if (teamMateData.numOfConnectedPlayers >= 2)
		isPlayerValid[teamMateData.secondTeamMate] = true;	

	if (teamMateData.numOfConnectedPlayers >= 3)
		isPlayerValid[teamMateData.thirdTeamMate] = true;	

	bool isPn3Connected = isPlayerValid[3];

	// Deactivate not-seeing-ball player & keeper
	for (int p = 1; p < TeamMateData::numOfPlayers; p++) {
		unsigned timeTMateBallSinceLastSeen = teamMateData.timeStamps[p] - teamMateData.ballModels[p].timeWhenLastSeen;
		if (isPlayerValid[p] && timeTMateBallSinceLastSeen > 3000) {
			isPlayerValid[p] = false;
		}
	}
	if (frameInfo.time - ballModel.timeWhenLastSeen > 3000) {
		isPlayerValid[robotInfo.number] = false;
	}
	isPlayerValid[1] = false;


	// Determine nearest player
	// v = 2(m) / 10(s)
	// w = 360(degree) / 8(s)
	for (int p = 1; p < TeamMateData::numOfPlayers; p++) {
		distToBallTMate[p] = teamMateData.ballModels[p].estimate.position.abs() / 200 + fabs(teamMateData.ballModels[p].estimate.position.angle()) / 0.785;
	}
	distToBallTMate[robotInfo.number] = ballModel.estimate.position.abs() / 200 + fabs(ballModel.estimate.position.angle()) / 0.785;


	int nearestPn = -1;
	double minDistToBall = 990000;

	for (int p = 1; p < TeamMateData::numOfPlayers; p++) {
		if (isPlayerValid[p] && distToBallTMate[p] < minDistToBall) {
			minDistToBall = distToBallTMate[p];
			nearestPn = p;
		}		
	}

	// Buffer to Switching
	if (nearestPn != strikerPn) {
		countSwitch++;
	} else {
		countSwitch = 0;
	}

	if (nearestPn == robotInfo.number || countSwitch > 40) {
		countSwitch = 0;
		strikerPn = nearestPn;
	}

	//if (frameInfo.time - ballModel.timeWhenLastSeen < 500 && ballModel.estimate.position.abs() < 500) {
	//	strikerPn = robotInfo.number;
	//}

	if (strikerPn == -1) {
		strikerPn = robotInfo.number;
	}

	// Visualization
/*
	static int displayCount;

	if (displayCount % 10 == 0) {
		system("clear");
		printf("Num Players = %d\n", TeamMateData::numOfPlayers);
		printf("PN = %d, time = %d, ballLastSeen = %d\n", robotInfo.number, frameInfo.time, ballModel.timeWhenLastSeen);
		printf("Num = %d\n", teamMateData.numOfConnectedPlayers);
		printf("1st>> %d\n", teamMateData.firstTeamMate);
		printf("2nd>> %d\n", teamMateData.secondTeamMate);
		printf("3rd>> %d\n", teamMateData.thirdTeamMate);

		printf("\n================================================\n\n");
		//for (unsigned int i = 0; i < vecNonKeeperPlayers.size(); i++) {
		//	printf("%d ", vecNonKeeperPlayers[i]);
		//}
		//printf("\n");

		for (int p = 1; p < TeamMateData::numOfPlayers; p++) {
			if (isPlayerValid[p]) {
				printf("O > ");
			} else {
				printf("X > ");				
			}
			
			printf(" %f ", distToBallTMate[p]);
			//printf("TimeLastSeen = %d, TimeSinceLastSeen = %d, TimeTMate = %d", timeTMateBallLastSeen[p], timeTMateBallSinceLastSeen[p], teamMateData.timeStamps[p]);
			printf("\n");			
		}
		printf("\n");

		printf("Nearest Pn = %d\n", nearestPn);
		printf("Striker Pn = %.1f\n", strikerPn);

		for (int p = 1; p < TeamMateData::numOfPlayers; p++) {
			printf("%d> %f, %f\n", p, teamMateData.ballModels[p].estimate.position.abs(), fabs(teamMateData.ballModels[p].estimate.position.angle()));			
		}
		printf("%d> %f, %f\n", robotInfo.number, ballModel.estimate.position.abs(), fabs(ballModel.estimate.position.angle()));	
	}
	displayCount++;	
*/

}

void BH2011BallSymbols::updateRole()
{
	if (robotInfo.number == 1) {
		return;
	}

	updateStrikerPn();

	if (robotInfo.number == strikerPn) {
		return;
	}

	// Supporter
	for (int p = 1; p < TeamMateData::numOfPlayers; p++) {
		isPlayerValid[p] = false;
	}

	// Activate self & connected teammates
	isPlayerValid[robotInfo.number] = true;		
	if (teamMateData.numOfConnectedPlayers >= 1)
		isPlayerValid[teamMateData.firstTeamMate] = true;	
	
	if (teamMateData.numOfConnectedPlayers >= 2)
		isPlayerValid[teamMateData.secondTeamMate] = true;	

	if (teamMateData.numOfConnectedPlayers >= 3)
		isPlayerValid[teamMateData.thirdTeamMate] = true;

	isPlayerValid[1] = false;
	isPlayerValid[(int)strikerPn] = false;

	int numSupporterCandidate = 0;
	int candidate1 = 0;
	int candidate2 = 0;

	for (int p = 1; p < TeamMateData::numOfPlayers; p++) {
		if (isPlayerValid[p]) {
			numSupporterCandidate++;

			if (candidate1 == 0) {
				candidate1 = p;
			} else {
				candidate2 = p;
			}
		}
	}

	

	float osp, dsp;
	if (numSupporterCandidate == 0) {
		offensiveSupporterPn = -1;
		defensiveSupporterPn = -1;

	} else if (numSupporterCandidate == 1) {
		double cx = candidate1 == robotInfo.number ? (robotPose.translation.x) : (teamMateData.robotPoses[candidate1].translation.x);
		
		if (cx < 0) {
			osp = -1;
			dsp = candidate1;		
		} else {
			osp = candidate1;
			dsp = -1;
		}
	} else {
		double c1x, c2x;

		c1x = candidate1 == robotInfo.number ? (robotPose.translation.x) : (teamMateData.robotPoses[candidate1].translation.x);
		c2x = candidate2 == robotInfo.number ? (robotPose.translation.x) : (teamMateData.robotPoses[candidate2].translation.x);			

		if (c1x < c2x) {
			dsp = candidate1;
			osp = candidate2;
		} else {
			dsp = candidate2;
			osp = candidate1;
		}	
	}

	if (osp != offensiveSupporterPn || dsp != defensiveSupporterPn) {
		countSupporterSwitch++;
	} else {
		countSupporterSwitch = 0;
	}

	if (countSupporterSwitch > 10) {
		offensiveSupporterPn = osp;
		defensiveSupporterPn = dsp;
	}

	//printf("%d> %f, %f, %f, %f\n", numSupporterCandidate, osp, dsp, offensiveSupporterPn, defensiveSupporterPn);
}

/*bool BH2011BallSymbols::updateBallTeam()
{
	// Compute ball positions in field from TMates
	for (int k = 0; k < TeamMateData::numOfPlayers; k++) {
		teamBallPositionsField[k] = teamMateData.ballModels[k].estimate.getPositionInFieldCoordinates(teamMateData.robotPoses[k]);
		poseTMate[k] = teamMateData.robotPoses[k];
		headTMate[k] = teamMateData.headMotionInfos[k];
	}

	// Update grids
	const int BallMax = 40;
	const int BallSeen = 40;
	const int BallFade = 1;
	const int BallUnseen = 25;	

	Vector2<> ballCLAT(stateEstimate.stateVector(13,1), stateEstimate.stateVector(15,1));

	for (int i = 0; i < BallGridSizeX; i++) {
		for (int j = 0; j < BallGridSizeY; j++) {
			if (ballGrid[i][j] < 0) {
				ballGrid[i][j] = min(ballGrid[i][j]+BallFade, BallMax);
			} else if (ballGrid[i][j] > 0) {
				ballGrid[i][j] = max(ballGrid[i][j]-BallFade, -BallMax);
			}

			Vector2 <> grid(i * (6000 / BallGridSizeX) - 3000, j * (4000 / BallGridSizeY) - 2000);
			
			// Ball by CLAT
			if (stateEstimate.isBallInit && (grid-ballCLAT).abs() < (6000 / BallGridSizeX)) {
				ballGrid[i][j] = min(ballGrid[i][j]+BallSeen, BallMax);
			}
			
			// Ball seen by self
			if (ballWasSeen && (grid-ballPositionField).abs() < (6000 / BallGridSizeX)) {
				ballGrid[i][j] = min(ballGrid[i][j]+BallSeen, BallMax);
			}

			// Ball seen by TMates, for MHT effect
			for (int k = 0; k < TeamMateData::numOfPlayers; k++) {
				if (teamMateData.ballModels[k].timeWhenLastSeen != ballTMateTime[k]) {					
					if ((grid-teamBallPositionsField[k]).abs() < (6000 / BallGridSizeX)) {
						ballGrid[i][j] = min(ballGrid[i][j]+BallSeen, BallMax);
					}
				}
			}

			// Negative Info by self
			double theta = (grid-robotPose.translation).angle()-robotPose.rotation-headMotionInfo.headPan;
			double r = (grid-robotPose.translation).abs();
			if (fabs(theta) < fromDegrees(23) && r < min(headMotionInfo.obserRangeMax,4500.0) && r > headMotionInfo.obserRangeMin) {
				ballGrid[i][j] = max(ballGrid[i][j]-BallUnseen, -BallMax);
			}

			
			// Negative Info by TMates
			for (int k = 0; k < TeamMateData::numOfPlayers; k++) {
				if (teamMateData.timeStamps[k] != poseTMateTime[k]) {					
					theta = (grid-poseTMate[k].translation).angle()-poseTMate[k].rotation-headTMate[k].headPan;
					r = (grid-poseTMate[k].translation).abs();
					if (fabs(theta) < fromDegrees(23) && r < min(headTMate[k].obserRangeMax,4500.0) && r > headTMate[k].obserRangeMin) {
						ballGrid[i][j] = max(ballGrid[i][j]-BallUnseen, -BallMax);
					}
				}
			}
		}
	}

	// Find max probable grid
	int maxProb = -40;	
	for (int i = 0; i < BallGridSizeX; i++) {
		for (int j = 0; j < BallGridSizeY; j++) {
			if (ballGrid[i][j] > maxProb) {
				maxProb = ballGrid[i][j];
			}
		}
	}

	if (2>1) {
		int countMax = 0;
		int imax, jmax;
		for (int i = 0; i < BallGridSizeX; i++) {
			for (int j = 0; j < BallGridSizeY; j++) {
				if (ballGrid[i][j] == maxProb) {
					countMax++;
					imax = i;
					jmax = j;
				}
			}
		}
				
		int rndCount = rand() % countMax + 1;
		for (int i = 0; i < BallGridSizeX; i++) {
			for (int j = 0; j < BallGridSizeY; j++) {
				if (ballGrid[i][j] == maxProb) {
					if (countMax == rndCount) {
						imax = i;
						jmax = j;
					}
					countMax--;
				}
			}
		}

		ballPositionTeamField.x = imax * (6000 / BallGridSizeX) - 3000;
		ballPositionTeamField.y = jmax * (4000 / BallGridSizeY) - 2000;
	}

	// Update TMate estimate time stamp	
	int dTimeTMate;

	for (int k = 0; k < TeamMateData::numOfPlayers; k++) {
		if (teamMateData.ballModels[k].timeWhenLastSeen != ballTMateTime[k]) {				
			ballTMateTime[k] = teamMateData.ballModels[k].timeWhenLastSeen;		
		}
		if (teamMateData.timeStamps[k] != 0) {
			dTimeTMate = teamMateData.timeStamps[k] - poseTMateTime[k];					
		}
		poseTMateTime[k] = teamMateData.timeStamps[k];	
	}

	// Visualization
	/*
	static int time0 = 0;	
	static int displayCount;

	if (displayCount % 10 == 0) {
		system("clear");
		//printf("\n= = = = = = = = = =\n");
		for (int i = BallGridSizeY-1; i >= 0; i--) {
			printf("| ");
			for (int j = 0; j < BallGridSizeX; j++) {
				printf("%01d ",ballGrid[j][i]/12+5);
			}
			printf("|\n");
		}		
		//printf("= = = = = = = = = =\n");

		printf("dTime = %f, dTimeTMate = %f\n", (frameInfo.time-time0)/1000.0, (double)dTimeTMate/1000.0);
		printf("[%d] Max Ball = (%f, %f)\n", (maxProb > 0), ballPositionTeamField.x, ballPositionTeamField.y);
		
		printf("Connect P: %d\n", teamMateData.numOfConnectedPlayers);
		for (int k = 0; k < TeamMateData::numOfPlayers; k++) {
			printf("P: %d, Time: %d\n", k, teamMateData.timeStamps[k]);
		}
	}
	displayCount++;		
	time0 = frameInfo.time;
	
	
	
	return (maxProb > 0) && ((ballPositionTeamField - robotPose.translation).abs() > 250);
}*/