/**
* @file TeamDataProvider.cpp
* This file implements a module that provides the data received by team communication.
* @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
*/

#include "TeamDataProvider.h"
#include "Tools/Settings.h"
#include "Tools/Team.h"
#include "Tools/Debugging/ReleaseOptions.h"

/**
 * This macro unpacks compressed representations. It reads
 * representationCompressed from the MessageQueue and unpacks it into
 * teamMateData.array[robotNumber].
 */
#define UNPACK(representation, array) \
  representation##Compressed the##representation##Compressed; \
  message.bin >> the##representation##Compressed; \
  theTeamMateData.array[robotNumber] = the##representation##Compressed.unpack();

/**
 * This macro converts a timeStamp into local time via ntp.
 */
#define REMOTE_TO_LOCAL_TIME(timeStamp) \
  if(timeStamp) { timeStamp = ntp.getRemoteTimeInLocalTime(timeStamp); }

PROCESS_WIDE_STORAGE(TeamDataProvider) TeamDataProvider::theInstance = 0;

TeamDataProvider::TeamDataProvider() :
  timeStamp(0),
  robotNumber(-1),
  lastSentTimeStamp(0)
{
  theInstance = this;
}

TeamDataProvider::~TeamDataProvider()
{
  theInstance = 0;
}

void TeamDataProvider::update(TeamMateData& teamMateData)
{
  DECLARE_PLOT("module:TeamDataProvider:sslVisionOffset");
  DECLARE_PLOT("module:TeamDataProvider:ntpOffset1"); // 1-4: players
  DECLARE_PLOT("module:TeamDataProvider:ntpOffset2");
  DECLARE_PLOT("module:TeamDataProvider:ntpOffset3");
  DECLARE_PLOT("module:TeamDataProvider:ntpOffset4");
  DECLARE_PLOT("module:TeamDataProvider:ntpOffset12"); // SSL vision data

  PLOT("module:TeamDataProvider:ntpOffset1", ntp.timeSyncBuffers[1].bestOffset);
  PLOT("module:TeamDataProvider:ntpOffset2", ntp.timeSyncBuffers[2].bestOffset);
  PLOT("module:TeamDataProvider:ntpOffset3", ntp.timeSyncBuffers[3].bestOffset);
  PLOT("module:TeamDataProvider:ntpOffset4", ntp.timeSyncBuffers[4].bestOffset);
  PLOT("module:TeamDataProvider:ntpOffset12", ntp.timeSyncBuffers[12].bestOffset);

  teamMateData = theTeamMateData;
  teamMateData.numOfConnectedPlayers = 0;
  teamMateData.firstTeamMate = teamMateData.secondTeamMate = teamMateData.thirdTeamMate = TeamMateData::numOfPlayers;
  for(int i = TeamMateData::firstPlayer; i < TeamMateData::numOfPlayers; ++i)
    if(teamMateData.timeStamps[i] && theFrameInfo.getTimeSince(teamMateData.timeStamps[i]) < networkTimeout)
    {
      if(!teamMateData.isPenalized[i] && teamMateData.hasGroundContact[i])
      {
        ++teamMateData.numOfConnectedPlayers;
        if(teamMateData.numOfConnectedPlayers == 1) teamMateData.firstTeamMate = i;
        else if(teamMateData.numOfConnectedPlayers == 2) teamMateData.secondTeamMate = i;
        else if(teamMateData.numOfConnectedPlayers == 3) teamMateData.thirdTeamMate = i;
      }
      else
        teamMateData.timeStamps[i] = 0;
    }
  if(teamMateData.numOfConnectedPlayers > 0)
    teamMateData.wasConnected = theTeamMateData.wasConnected = true;
  teamMateData.sendThisFrame = (ntp.doSynchronization(theFrameInfo.time, Global::getTeamOut()) ||
                                theFrameInfo.getTimeSince(lastSentTimeStamp) >= 100) // TODO config file?
#ifdef TARGET_ROBOT
                               && !(theMotionRequest.motion == MotionRequest::specialAction && theMotionRequest.specialActionRequest.specialAction == SpecialActionRequest::playDead)
                               && !(theMotionInfo.motion == MotionRequest::specialAction && theMotionInfo.specialActionRequest.specialAction == SpecialActionRequest::playDead)
#endif
                               ;
  if(teamMateData.sendThisFrame)
  {
    lastSentTimeStamp = theFrameInfo.time;
    TEAM_OUTPUT(idTeamMateIsPenalized, bin, (theRobotInfo.penalty != PENALTY_NONE));
    TEAM_OUTPUT(idTeamMateHasGroundContact, bin, (theGroundContactState.contact || !theDamageConfiguration.useGroundContactDetection));
    TEAM_OUTPUT(idTeamMateIsUpright, bin, (theFallDownState.state == theFallDownState.upright));
    if(theGroundContactState.contact || !theDamageConfiguration.useGroundContactDetection)
      TEAM_OUTPUT(idTeamMateTimeSinceLastGroundContact, bin, lastSentTimeStamp);
  }
}

void TeamDataProvider::handleMessages(MessageQueue& teamReceiver)
{
  if(theInstance)
  {
    teamReceiver.handleAllMessages(*theInstance);
    TEAM_OUTPUT(idRobot, bin, theInstance->theRobotInfo.number);
  }

  teamReceiver.clear();
}

bool TeamDataProvider::handleMessage(InMessage& message)
{
  /*
  The robotNumber and the three flags hasGroundContact, isUpright and isPenalized should always be updated.
  */
  switch(message.getMessageID())
  {
  case idNTPHeader:
    VERIFY(ntp.handleMessage(message));
    timeStamp = ntp.receiveTimeStamp;
    return false;
  case idNTPIdentifier:
  case idNTPRequest:
  case idNTPResponse:
    return ntp.handleMessage(message);

  case idRobot:
    message.bin >> robotNumber;
    if(robotNumber != theRobotInfo.number)
      if(robotNumber >= TeamMateData::firstPlayer && robotNumber < TeamMateData::numOfPlayers)
        theTeamMateData.timeStamps[robotNumber] = timeStamp;
    return true;

  case idReleaseOptions:
    message.bin >> Global::getReleaseOptions();
    return true;

  case idSSLVisionData:
  {
    message.bin >> theSSLVisionData;
    unsigned remoteTimestamp = theSSLVisionData.recentData.top().receiveTimestamp;
    REMOTE_TO_LOCAL_TIME(theSSLVisionData.recentData.top().receiveTimestamp);
    unsigned localTimestamp = theSSLVisionData.recentData.top().receiveTimestamp;
    int offset = (int) localTimestamp - (int) remoteTimestamp;
    PLOT("module:TeamDataProvider:sslVisionOffset", offset);
  }
  return true;

  case idGroundTruthBallModel:
  {
    Vector2<> position;
    message.bin >> theGroundTruthBallModel.timeWhenLastSeen >> position;
    REMOTE_TO_LOCAL_TIME(theGroundTruthBallModel.timeWhenLastSeen);
    if(theOwnTeamInfo.teamColor == TEAM_BLUE)
      position *= -1;
    theGroundTruthBallModel.lastPerception.setPositionAndVelocityInFieldCoordinates(
      position, Vector2<>(), theGroundTruthRobotPose);
    theGroundTruthBallModel.estimate = theGroundTruthBallModel.lastPerception;
  }
  return true;

  case idGroundTruthRobotPose:
  {
    char teamColor,
         id;
    unsigned timeStamp;
    Pose2D robotPose;
    message.bin >> teamColor >> id >> timeStamp >> robotPose;
    if(teamColor == (int)theOwnTeamInfo.teamColor && id == theRobotInfo.number)
    {
      if(theOwnTeamInfo.teamColor == TEAM_BLUE)
        robotPose = Pose2D(pi) + robotPose;
      (Pose2D&) theGroundTruthRobotPose = robotPose;
    }
  }
  return true;

  case idTeamMateIsPenalized:
    if(robotNumber != theRobotInfo.number)
      if(robotNumber >= TeamMateData::firstPlayer && robotNumber < TeamMateData::numOfPlayers)
        message.bin >> theTeamMateData.isPenalized[robotNumber];
    return true;

  case idTeamMateHasGroundContact:
    if(robotNumber != theRobotInfo.number)
      if(robotNumber >= TeamMateData::firstPlayer && robotNumber < TeamMateData::numOfPlayers)
        message.bin >> theTeamMateData.hasGroundContact[robotNumber];
    return true;

  case idTeamMateIsUpright:
    if(robotNumber != theRobotInfo.number)
      if(robotNumber >= TeamMateData::firstPlayer && robotNumber < TeamMateData::numOfPlayers)
        message.bin >> theTeamMateData.isUpright[robotNumber];
    return true;

  case idTeamMateTimeSinceLastGroundContact:
    if(robotNumber != theRobotInfo.number)
      if(robotNumber >= TeamMateData::firstPlayer && robotNumber < TeamMateData::numOfPlayers)
      {
        message.bin >> theTeamMateData.timeSinceLastGroundContact[robotNumber];
        REMOTE_TO_LOCAL_TIME(theTeamMateData.timeSinceLastGroundContact[robotNumber]);
      }
    return true;

  default:
    break;
  }

  /*
  The messages in the following switch block should only be updated
  if hasGroundContact == true and isPenalized == false, because the information of this message
  can only be reliable if the robot is actively playing.
  */
  if(!theTeamMateData.isPenalized[robotNumber] && theTeamMateData.hasGroundContact[robotNumber])
  {
    switch(message.getMessageID())
    {
    case idTeamMateBallAfterKickPose:
    {
      BallAfterKickPose bakp;
      message.bin >> bakp;
      REMOTE_TO_LOCAL_TIME(bakp.timeWhenLastKickWasPerformed);
      if(theFrameInfo.getTimeSince(bakp.timeWhenLastKickWasPerformed) == 0 || theFrameInfo.getTimeSince(theTeamMateData.ballAfterKickPose.timeWhenLastKickWasPerformed) >
         theFrameInfo.getTimeSince(bakp.timeWhenLastKickWasPerformed))
        theTeamMateData.ballAfterKickPose = bakp;
      return true;
    }

    case idTeamMatePassTarget:
      message.bin >> theTeamMateData.passTarget;
      return true;

    case idTeamMateBallModel:
      if(robotNumber != theRobotInfo.number)
        if(robotNumber >= TeamMateData::firstPlayer && robotNumber < TeamMateData::numOfPlayers)
        {
          UNPACK(BallModel, ballModels);
          REMOTE_TO_LOCAL_TIME(theTeamMateData.ballModels[robotNumber].timeWhenLastSeen);
        }
      return true;

    case idTeamMateBallHypotheses:
      if(robotNumber != theRobotInfo.number)
        if(robotNumber >= TeamMateData::firstPlayer && robotNumber < TeamMateData::numOfPlayers)
        {
          BallHypotheses& ballHypotheses = theTeamMateData.ballHypotheses[robotNumber];
          message.bin >> ballHypotheses;
          REMOTE_TO_LOCAL_TIME(ballHypotheses.timeWhenDisappeared);
        }
      return true;

    case idTeamMateObstacleModel:
      if(robotNumber != theRobotInfo.number)
        if(robotNumber >= TeamMateData::firstPlayer && robotNumber < TeamMateData::numOfPlayers)
        {
          ObstacleModel& obstacleModel = theTeamMateData.obstacleModels[robotNumber];
          message.bin >> obstacleModel;
        }
      return true;

    case idTeamMateRobotsModel:
      if(robotNumber != theRobotInfo.number)
        if(robotNumber >= TeamMateData::firstPlayer && robotNumber < TeamMateData::numOfPlayers)
        {
          UNPACK(RobotsModel, robotsModels);
          for(size_t i = 0; i < theTeamMateData.robotsModels[robotNumber].robots.size(); i++)
          {
            REMOTE_TO_LOCAL_TIME(theTeamMateData.robotsModels[robotNumber].robots[i].timeStamp);
          }
        }
      return true;

    case idTeamMateRobotPose:
      if(robotNumber != theRobotInfo.number)
        if(robotNumber >= TeamMateData::firstPlayer && robotNumber < TeamMateData::numOfPlayers)
        {
          UNPACK(RobotPose, robotPoses);
        }
      return true;

    case idTeamMateBehaviorData:
      if(robotNumber != theRobotInfo.number)
        if(robotNumber >= TeamMateData::firstPlayer && robotNumber < TeamMateData::numOfPlayers)
          message.bin >> theTeamMateData.behaviorData[robotNumber];
      return true;

    case idTeamHeadControl:
      if(robotNumber != theRobotInfo.number)
        if(robotNumber >= TeamMateData::firstPlayer && robotNumber < TeamMateData::numOfPlayers)
          message.bin >> theTeamMateData.teamHeadControlStates[robotNumber];
      return true;

    case idTeamCameraHeight:
      if(robotNumber != theRobotInfo.number)
        if(robotNumber >= TeamMateData::firstPlayer && robotNumber < TeamMateData::numOfPlayers)
          message.bin >> theTeamMateData.cameraHeights[robotNumber];
      return true;

    case idTeamMateFieldCoverage:
      if(robotNumber != theRobotInfo.number)
        if(robotNumber >= TeamMateData::firstPlayer && robotNumber < TeamMateData::numOfPlayers)
        {
          FieldCoverage::GridInterval& gridInterval = theTeamMateData.fieldCoverages[robotNumber];
          message.bin >> gridInterval;
          REMOTE_TO_LOCAL_TIME(gridInterval.timestamp);
        }
      return true;

    default:
      break;
    }
  }

  return true;
}

MAKE_MODULE(TeamDataProvider, Infrastructure)
