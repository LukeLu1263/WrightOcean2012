/**
* @file MotionLogDataProvider.cpp
* This file implements a module that provides data replayed from a log file.
* @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas R�fer</a>
*/

#include "MotionLogDataProvider.h"
#include "Tools/Debugging/DebugDrawings.h"

PROCESS_WIDE_STORAGE(MotionLogDataProvider) MotionLogDataProvider::theInstance = 0;


MotionLogDataProvider::MotionLogDataProvider() :
  LogDataProvider(), frameDataComplete(false)
{
  theInstance = this;
}

MotionLogDataProvider::~MotionLogDataProvider()
{
  theInstance = 0;
}


void MotionLogDataProvider::update(GroundTruthOdometryData& groundTruthOdometryData)
{
  if(representationBuffer[idOdometryData])
    groundTruthOdometryData = *((GroundTruthOdometryData*)representationBuffer[idOdometryData]);
#ifndef RELEASE
  Pose2D odometryOffset(groundTruthOdometryData);
  odometryOffset -= lastOdometryData;
  PLOT("module:MotionLogDataProvider:odometryOffsetX", odometryOffset.translation.x);
  PLOT("module:MotionLogDataProvider:odometryOffsetY", odometryOffset.translation.y);
  PLOT("module:MotionLogDataProvider:odometryOffsetRotation", toDegrees(odometryOffset.rotation));
  lastOdometryData = groundTruthOdometryData;
#endif
}

void MotionLogDataProvider::update(GroundTruthOrientationData& groundTruthOrientationData)
{
  if(representationBuffer[idOrientationData])
    groundTruthOrientationData = *((GroundTruthOrientationData*)representationBuffer[idOrientationData]);
#ifndef RELEASE
  PLOT("module:MotionLogDataProvider:gtOrientationX", toDegrees(groundTruthOrientationData.orientation.x));
  PLOT("module:MotionLogDataProvider:gtOrientationY", toDegrees(groundTruthOrientationData.orientation.y));
  PLOT("module:MotionLogDataProvider:gtVelocityX", groundTruthOrientationData.velocity.x);
  PLOT("module:MotionLogDataProvider:gtVelocityY", groundTruthOrientationData.velocity.y);
  PLOT("module:MotionLogDataProvider:gtVelocityZ", groundTruthOrientationData.velocity.z);
#endif
}

bool MotionLogDataProvider::handleMessage(InMessage& message)
{
  return theInstance && theInstance->handleMessage2(message);
}

bool MotionLogDataProvider::isFrameDataComplete()
{
  if(!theInstance)
    return true;
  else if(theInstance->frameDataComplete)
  {
    OUTPUT(idLogResponse, bin, '\0');
    theInstance->frameDataComplete = false;
    return true;
  }
  else
    return false;
}

bool MotionLogDataProvider::handleMessage2(InMessage& message)
{
  switch(message.getMessageID())
  {
    HANDLE(BoardInfo)
    HANDLE2(JointData,
    {
      ALLOC(FrameInfo);
      FrameInfo& frameInfo = (FrameInfo&) *representationBuffer[idFrameInfo];
      JointData& jointData = (JointData&) *representationBuffer[idJointData];
      frameInfo.time = jointData.timeStamp;
      frameInfo.cycleTime = jointData.cycleTime;
    })
    HANDLE(KeyStates)
    HANDLE(OdometryData)
    HANDLE(SensorData)
    HANDLE(FilteredSensorData)
    HANDLE(OrientationData)

  case idProcessFinished:
    frameDataComplete = true;
    return true;

  default:
    return false;
  }
}

MAKE_MODULE(MotionLogDataProvider, Infrastructure)
