/**
* @file CognitionLogDataProvider.h
* This file declares a module that provides data replayed from a log file.
* @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas R�fer</a>
*/

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Infrastructure/Image.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Perception/LinePercept.h"
#include "Representations/Perception/BallPercept.h"
#include "Representations/Perception/GoalPercept.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/Perception/ImageCoordinateSystem.h"
#include "Representations/Perception/ImageInfo.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/RobotsModel.h"
#include "Tools/MessageQueue/InMessage.h"
#include "Tools/Debugging/DebugImages.h"
#include "LogDataProvider.h"
#include "Tools/Team.h"
#include "Tools/Debugging/DebugDrawings3D.h"
#include "Representations/Perception/JPEGImage.h"


MODULE(CognitionLogDataProvider)
  PROVIDES_WITH_OUTPUT(Image)
  PROVIDES(CameraInfo)
  PROVIDES_WITH_MODIFY_AND_OUTPUT(FrameInfo)
  PROVIDES_WITH_MODIFY_AND_OUTPUT(LinePercept)
  PROVIDES_WITH_MODIFY_AND_OUTPUT_AND_DRAW(BallPercept)
  PROVIDES_WITH_MODIFY_AND_OUTPUT_AND_DRAW(GoalPercept)
  PROVIDES_WITH_MODIFY_AND_OUTPUT_AND_DRAW(RobotPose)
  PROVIDES_WITH_MODIFY_AND_OUTPUT_AND_DRAW(BallModel)
  PROVIDES_WITH_DRAW(RobotsModel)
  USES(FrameInfo)
  PROVIDES_WITH_MODIFY_AND_OUTPUT_AND_DRAW(GroundTruthRobotPose)
  PROVIDES_WITH_MODIFY_AND_OUTPUT_AND_DRAW(GroundTruthBallModel)
  PROVIDES_WITH_DRAW(GroundTruthRobotsModel)
  PROVIDES_WITH_MODIFY_AND_OUTPUT_AND_DRAW(CameraMatrix)
  REQUIRES(Image)
  PROVIDES_WITH_MODIFY_AND_OUTPUT(ImageCoordinateSystem)
  PROVIDES_WITH_MODIFY_AND_OUTPUT(ImageInfo)
END_MODULE

class CognitionLogDataProvider : public CognitionLogDataProviderBase, public LogDataProvider
{
private:
  PROCESS_WIDE_STORAGE_STATIC(CognitionLogDataProvider) theInstance; /**< Points to the only instance of this class in this process or is 0 if there is none. */
  bool frameDataComplete; /**< Were all messages of the current frame received? */

  DECLARE_DEBUG_IMAGE(corrected);

  void update(CameraInfo& cameraInfo) {}

#define DISTANCE 300
  UPDATE(ImageInfo)
  UPDATE(LinePercept)
  UPDATE2(RobotPose, TEAM_OUTPUT(idTeamMateRobotPose, bin, RobotPoseCompressed(_RobotPose));)
  UPDATE2(BallModel, TEAM_OUTPUT(idTeamMateBallModel, bin, BallModelCompressed(_BallModel));)
  UPDATE2(RobotsModel, TEAM_OUTPUT(idTeamMateRobotsModel, bin, RobotsModelCompressed(_RobotsModel));)

  UPDATE(BallPercept)
  UPDATE(CameraMatrix)
  UPDATE(FrameInfo)
  UPDATE(GoalPercept)
  UPDATE(GroundTruthBallModel)
  UPDATE2(GroundTruthRobotPose, _GroundTruthRobotPose.timestamp = theFrameInfo.time;)
  UPDATE(GroundTruthRobotsModel)

  UPDATE2(Image,
  {
    DECLARE_DEBUG_DRAWING3D("representation:Image", "camera");
    IMAGE3D("representation:Image", DISTANCE, 0, 0, 0, 0, 0,
            DISTANCE * _Image.cameraInfo.resolutionWidth / _Image.cameraInfo.focalLength,
            DISTANCE * _Image.cameraInfo.resolutionHeight / _Image.cameraInfo.focalLength,
            _Image);
    DEBUG_RESPONSE("representation:JPEGImage", OUTPUT(idJPEGImage, bin, JPEGImage(_Image)););
  })

  UPDATE2(ImageCoordinateSystem,
  {
    DECLARE_DEBUG_DRAWING("loggedHorizon", "drawingOnImage"); // displays the horizon
    ARROW("loggedHorizon",
          _ImageCoordinateSystem.origin.x,
          _ImageCoordinateSystem.origin.y,
          _ImageCoordinateSystem.origin.x + _ImageCoordinateSystem.rotation.c[0].x * 50,
          _ImageCoordinateSystem.origin.y + _ImageCoordinateSystem.rotation.c[0].y * 50,
          0, Drawings::ps_solid, ColorRGBA(255, 0, 0));
    ARROW("loggedHorizon",
          _ImageCoordinateSystem.origin.x,
          _ImageCoordinateSystem.origin.y,
          _ImageCoordinateSystem.origin.x + _ImageCoordinateSystem.rotation.c[1].x * 50,
          _ImageCoordinateSystem.origin.y + _ImageCoordinateSystem.rotation.c[1].y * 50,
          0, Drawings::ps_solid, ColorRGBA(255, 0, 0));
    COMPLEX_DEBUG_IMAGE(corrected,
    {
      Image* i = (Image*) representationBuffer[idImage];
      if(i)
      {
        INIT_DEBUG_IMAGE_BLACK(corrected);
        int yDest = -_ImageCoordinateSystem.toCorrectedCenteredNeg(0, 0).y;
        for(int ySrc = 0; ySrc < i->cameraInfo.resolutionHeight; ++ySrc)
          for(int yDest2 = -_ImageCoordinateSystem.toCorrectedCenteredNeg(0, ySrc).y; yDest <= yDest2; ++yDest)
          {
            int xDest = -_ImageCoordinateSystem.toCorrectedCenteredNeg(0, ySrc).x;
            for(int xSrc = 0; xSrc < i->cameraInfo.resolutionWidth; ++xSrc)
            {
              for(int xDest2 = -_ImageCoordinateSystem.toCorrectedCenteredNeg(xSrc, ySrc).x; xDest <= xDest2; ++xDest)
              {
                DEBUG_IMAGE_SET_PIXEL_YUV(corrected, xDest + int(i->cameraInfo.opticalCenter.x + 0.5f),
                yDest + int(i->cameraInfo.opticalCenter.y + 0.5f),
                i->image[ySrc][xSrc].y,
                i->image[ySrc][xSrc].cb,
                i->image[ySrc][xSrc].cr);
              }
            }
          }
        SEND_DEBUG_IMAGE(corrected);
      }
    });
  })

  /**
  * The method is called for every incoming debug message by handleMessage.
  * @param message An interface to read the message from the queue.
  * @return Was the message handled?
  */
  bool handleMessage2(InMessage& message);

public:
  /**
  * Default constructor.
  */
  CognitionLogDataProvider();

  /**
  * Destructor.
  */
  ~CognitionLogDataProvider();

  /**
  * The method is called for every incoming debug message.
  * @param message An interface to read the message from the queue.
  * @return Was the message handled?
  */
  static bool handleMessage(InMessage& message);

  /**
  * The method returns whether idProcessFinished was received.
  * @return Were all messages of the current frame received?
  */
  static bool isFrameDataComplete();
};
