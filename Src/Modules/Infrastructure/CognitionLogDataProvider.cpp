/**
* @file CognitionLogDataProvider.cpp
* This file implements a module that provides data replayed from a log file.
* @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas R�fer</a>
*/

#include "CognitionLogDataProvider.h"

PROCESS_WIDE_STORAGE(CognitionLogDataProvider) CognitionLogDataProvider::theInstance = 0;

#define ASSIGN(target, source) \
  ALLOC(target) \
  (target&) *representationBuffer[id##target] = (target&) *representationBuffer[id##source];

CognitionLogDataProvider::CognitionLogDataProvider() :
  LogDataProvider(),
  frameDataComplete(false)
{
  theInstance = this;
  ImageCoordinateSystem::initTables(CameraInfo());
}

CognitionLogDataProvider::~CognitionLogDataProvider()
{
  theInstance = 0;
}

bool CognitionLogDataProvider::handleMessage(InMessage& message)
{
  return theInstance && theInstance->handleMessage2(message);
}

bool CognitionLogDataProvider::isFrameDataComplete()
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

bool CognitionLogDataProvider::handleMessage2(InMessage& message)
{
  switch(message.getMessageID())
  {
    HANDLE(ImageInfo)
    HANDLE(BallModel)
    HANDLE(BallPercept)
    HANDLE(CameraMatrix)
    HANDLE(FrameInfo)
    HANDLE(GoalPercept)
    HANDLE2(GroundTruthRobotPose, ASSIGN(RobotPose, GroundTruthRobotPose))
    HANDLE2(GroundTruthBallModel, ASSIGN(BallModel, GroundTruthBallModel))
    HANDLE2(GroundTruthRobotsModel, ASSIGN(RobotsModel, GroundTruthRobotsModel))

    HANDLE2(Image,
    {
      ALLOC(FrameInfo)
      ((FrameInfo&) *representationBuffer[idFrameInfo]).time = ((Image&) *representationBuffer[idImage]).timeStamp;
    })
    HANDLE(ImageCoordinateSystem)
    HANDLE(LinePercept)
    HANDLE(RobotPose)

  case idProcessFinished:
    frameDataComplete = true;
    return true;

  case idJPEGImage:
    ALLOC(Image)
    {
      JPEGImage jpegImage;
      message.bin >> jpegImage;
      jpegImage.toImage((Image&) *representationBuffer[idImage]);
    }
    ALLOC(FrameInfo)
    ((FrameInfo&) *representationBuffer[idFrameInfo]).time = ((Image&) *representationBuffer[idImage]).timeStamp;
    return true;

  default:
    return false;
  }
}

MAKE_MODULE(CognitionLogDataProvider, Infrastructure)
