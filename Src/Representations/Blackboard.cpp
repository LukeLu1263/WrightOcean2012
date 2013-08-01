/**
* @file Blackboard.cpp
* Implementation of a class representing the blackboard.
* @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas R�fer</a>
*/

#include "Blackboard.h"
#include <cstring>
#include <cstdlib>

Blackboard::Blackboard() :
// Initialize all representations by themselves:
// Infrastructure
  theJointData(theJointData),
  theJointRequest(theJointRequest),
  theSensorData(theSensorData),
  theKeyStates(theKeyStates),
  theLEDRequest(theLEDRequest),
  theImage(theImage),
  theCameraInfo(theCameraInfo),
  theFrameInfo(theFrameInfo),
  theCognitionFrameInfo(theCognitionFrameInfo),
  theRobotInfo(theRobotInfo),
  theOwnTeamInfo(theOwnTeamInfo),
  theOpponentTeamInfo(theOpponentTeamInfo),
  theGameInfo(theGameInfo),
  theSoundRequest(theSoundRequest),
  theSoundOutput(theSoundOutput),
  theTeamMateData(theTeamMateData),
  theMotionRobotHealth(theMotionRobotHealth),
  theRobotHealth(theRobotHealth),
  theBoardInfo(theBoardInfo),

// Configuration
  theColorTable64(theColorTable64),
  theCameraSettings(theCameraSettings),
  theFieldDimensions(theFieldDimensions),
  theRobotDimensions(theRobotDimensions),
  theJointCalibration(theJointCalibration),
  theSensorCalibration(theSensorCalibration),
  theCameraCalibration(theCameraCalibration),
  theBehaviorConfiguration(theBehaviorConfiguration),
  theMassCalibration(theMassCalibration),
  theHardnessSettings(theHardnessSettings),
  theDamageConfiguration(theDamageConfiguration),

// Perception
  theCameraMatrix(theCameraMatrix),
  theCameraMatrixOther(theCameraMatrixOther),
  theCameraMatrixPrev(theCameraMatrixPrev),
  theRobotCameraMatrix(theRobotCameraMatrix),
  theRobotCameraMatrixOther(theRobotCameraMatrixOther),
  theRobotCameraMatrixPrev(theRobotCameraMatrixPrev),
  theImageCoordinateSystem(theImageCoordinateSystem),
  theBallSpots(theBallSpots),
  theLineSpots(theLineSpots),
  theBallPercept(theBallPercept),
  theLinePercept(theLinePercept),
  theRegionPercept(theRegionPercept),
  theGoalPercept(theGoalPercept),
  theGroundContactState(theGroundContactState),
  theBodyContour(theBodyContour),
  theTeamMarkerSpots(theTeamMarkerSpots),
  theRobotPercept(theRobotPercept),
  theImageInfo(theImageInfo),
  theImageRequest(theImageRequest),

// Modeling
  theArmContactModel(theArmContactModel),
  theFallDownState(theFallDownState),
  theBallAfterKickPose(theBallAfterKickPose),
  theBallHypotheses(theBallHypotheses),
  theBallModel(theBallModel),
  theCombinedWorldModel(theCombinedWorldModel),
  theGroundTruthBallModel(theGroundTruthBallModel),
  theObstacleModel(theObstacleModel),
  theUSObstacleGrid(theUSObstacleGrid),
  theRobotPose(theRobotPose),
  theRobotPoseInfo(theRobotPoseInfo),
  thePotentialRobotPose(thePotentialRobotPose),
  theGroundTruthRobotPose(theGroundTruthRobotPose),
  theRobotPoseHypotheses(theRobotPoseHypotheses),
  theRobotsModel(theRobotsModel),
  theGroundTruthRobotsModel(theGroundTruthRobotsModel),
  theFreePartOfOpponentGoalModel(theFreePartOfOpponentGoalModel),
  theSSLVisionData(theSSLVisionData),
  theGroundTruthResult(theGroundTruthResult),
  theFieldCoverage(theFieldCoverage),
  theGlobalFieldCoverage(theGlobalFieldCoverage),

// BehaviorControl
  theBehaviorControlOutput(theBehaviorControlOutput),
  theHeadMotionProposal(theHeadMotionProposal),
  theKickInfo(theKickInfo),
  theBehaviorLEDRequest(theBehaviorLEDRequest),

// Sensing
  theFilteredJointData(theFilteredJointData),
  theFilteredJointDataPrev(theFilteredJointDataPrev),
  theFilteredSensorData(theFilteredSensorData),
  theInertiaSensorData(theInertiaSensorData),
  theInspectedInertiaSensorData(theInspectedInertiaSensorData),
  theOrientationData(theOrientationData),
  theGroundTruthOrientationData(theGroundTruthOrientationData),
  theTorsoMatrix(theTorsoMatrix),
  theRobotModel(theRobotModel),

// MotionControl
  theOdometryData(theOdometryData),
  theGroundTruthOdometryData(theGroundTruthOdometryData),
  theMotionRequest(theMotionRequest),
  theHeadAngleRequest(theHeadAngleRequest),
  theHeadMotionRequest(theHeadMotionRequest),
  theHeadJointRequest(theHeadJointRequest),
  theMotionSelection(theMotionSelection),
  theSpecialActionsOutput(theSpecialActionsOutput),
  theWalkingEngineOutput(theWalkingEngineOutput),
  theWalkingEngineStandOutput(theWalkingEngineStandOutput),
  theBikeEngineOutput(theBikeEngineOutput),
  theMotionInfo(theMotionInfo),
  theUnstableJointRequest(theUnstableJointRequest),

// Debugging
  theXabslInfo(theXabslInfo)
  {
  }

void Blackboard::operator=(const Blackboard& other)
{
  memcpy(this, &other, sizeof(Blackboard));
}

void* Blackboard::operator new(std::size_t size)
{
  return calloc(1, size);
}

void Blackboard::operator delete(void* p)
{
  return free(p);
}

void Blackboard::distract()
{
}

PROCESS_WIDE_STORAGE(Blackboard) Blackboard::theInstance = 0;
