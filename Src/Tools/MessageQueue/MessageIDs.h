/**
* @file MessageIDs.h
*
* Declaration of ids for debug messages.
*
* @author Martin L�tzsch
*/

#pragma once

#include "Tools/Enum.h"

/**
* IDs for debug messages
*
* To distinguish debug messages, they all have an id.
*/
ENUM(MessageID,
  undefined,
  idProcessBegin,
  idProcessFinished,

  // data (ids should remain constant over code changes, so old log files will still work)
  idImage,
  idJPEGImage,
  idJointData,
  idSensorData,
  idKeyStates,
  idOdometryData,
  idFrameInfo,
  idFilteredJointData,
  idLinePercept,
  idGoalPercept,
  idBallPercept,
  idGroundTruthBallModel,
  idGroundTruthRobotPose,
  idSSLVisionData,
  idCameraMatrix,
  idCameraInfo,
  idImageCoordinateSystem,
  idBoardInfo,
  idRobotPose,
  idBallModel,
  idFilteredSensorData,
  idImageInfo,
  idOrientationData,
  idGameInfo,
  idRobotInfo,
  idXabslDebugMessage,
  idXabslDebugSymbols,
  idRobotsModel,
  idGroundTruthRobotsModel,

  // insert new data ids here
  numOfDataMessageIDs, /**< everything below this does not belong into log files */

  // ids used in team communication
  idNTPHeader = numOfDataMessageIDs,
  idNTPIdentifier,
  idNTPRequest,
  idNTPResponse,
  idRobot,
  idReleaseOptions,
  idStopwatch,
  idTeamMateBallModel,
  idTeamMateBallHypotheses,
  idTeamMateObstacleModel,
  idTeamMateRobotPose,
  idTeamMateBehaviorData,
  idRobotHealth,
  idMotionRequest,
  idTeamMateGoalPercept,
  idTeamMateRobotsModel,
  idTeamMateFreePartOfOpponentGoalModel,
  idTeamMateIsPenalized,
  idTeamMateHasGroundContact,
  idTeamMateIsUpright,
  idTeamMateBallAfterKickPose,
  idTeamMatePassTarget,
  idTeamMateCombinedWorldModel,
  idTeamMateSSLVisionData,
  idTeamHeadControl,
  idTeamMateTimeSinceLastGroundContact,
  idTeamCameraHeight,
  idTeamMateFieldCoverage,
  // insert new team comm ids here

  // infrastructure
  idText,
  idDebugRequest,
  idDebugResponse,
  idDebugDataResponse,
  idDebugDataChangeRequest,
  idStreamSpecification,
  idModuleTable,
  idModuleRequest,
  idQueueFillRequest,
  idLogResponse,
  idDrawingManager,
  idDrawingManager3D,
  idDebugImage,
  idDebugJPEGImage,
  idDebugGrayScaleImage,
  idDebugColorClassImage,
  idDebugDrawing,
  idDebugDrawing3D,
  idColorTable64,
  idWriteColorTable64,
  idXabslDebugRequest,
  idXabslIntermediateCode,
  idMotionNet,
  idJointRequest,
  idLEDRequest,
  idPlot,
  idConsole,
  idRobotname,
  idRobotDimensions,
  idJointCalibration,
  idImageRequest,
  idWalkingEngineKick
);
