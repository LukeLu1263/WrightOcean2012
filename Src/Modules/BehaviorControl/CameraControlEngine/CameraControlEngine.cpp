/**
* @file CameraControlEngine.cpp
* @author Felix Wenk
*/

#include "Tools/Streams/InStreams.h"
#include "Tools/Math/Geometry.h"

#include "CameraControlEngine.h"

MAKE_MODULE(CameraControlEngine, Behavior Control);

CameraControlEngine::CameraControlEngine() : requestedLowerCamera(true), lastFrameLowerClipped(false)
{
  InBinaryFile mtaStream("minimumTiltAngle.tab");
  if(mtaStream.exists())
    mtaStream.read(minimumTiltAngle, sizeof(minimumTiltAngle));
  else
    for(int i = 0; i < 120; i++)
    {
      minimumTiltAngle[0][i] = minimumTiltAngle[1][i] = -1.;
      minimumDistances[0][i] = minimumDistances[1][i] = -1.;
    }

  InConfigFile parameterStream("cameraControlEngine.cfg");
  if(parameterStream.exists())
    parameterStream >> p;
}

void CameraControlEngine::calculateYImageCoordinate(const Vector3<>& hip2Target,
    const Pose3D& transformMatrix, const Pose3D& transformMatrixOther,
    float& yImage, float& yImageOther)
{
  const Pose3D transformMatrixInv = transformMatrix.invert();
  const Pose3D transformMatrixOtherInv = transformMatrixOther.invert();
  // We don't want points to be behind the camera. Since we only need the y image coordinate,
  // we pretend lies directly in front of the camera.
  const float absInPlane = sqrt(sqr(hip2Target.x) + sqr(hip2Target.y));
  const Vector3<> fakeTarget(absInPlane * cos(theFilteredJointData.angles[JointData::HeadYaw]),
                             absInPlane * sin(theFilteredJointData.angles[JointData::HeadYaw]),
                             hip2Target.z);
  const Vector3<> fakeTargetCamera = transformMatrixInv * fakeTarget;
  const Vector3<> fakeTargetCameraOther = transformMatrixOtherInv * fakeTarget;
  yImage = -fakeTargetCamera.z * theCameraInfo.focalLength / fakeTargetCamera.x;
  yImageOther = -fakeTargetCameraOther.z * theCameraInfo.focalLength / fakeTargetCameraOther.x;
}

float CameraControlEngine::calculateTiltAngles(const Vector3<>& hip2Target, bool lowerCamera)
{
  Vector2<> headJoint2Target(sqrt(sqr(hip2Target.x) + sqr(hip2Target.y)), hip2Target.z - theRobotDimensions.zLegJoint1ToHeadPan);
  Vector2<> headJoint2Camera(theRobotDimensions.getXHeadTiltToCamera(lowerCamera),
                             theRobotDimensions.getZHeadTiltToCamera(lowerCamera));
  const float headJoint2CameraAngle = atan2(headJoint2Camera.x, headJoint2Camera.y);
  const float cameraAngle = pi - (pi_2 - headJoint2CameraAngle) - theRobotDimensions.getHeadTiltToCameraTilt(lowerCamera);
  const float targetAngle = asin(headJoint2Camera.abs() * sin(cameraAngle) / headJoint2Target.abs());
  const float headJointAngle = pi - targetAngle - cameraAngle;
  const float tilt = atan2(headJoint2Target.x, headJoint2Target.y) - headJointAngle - headJoint2CameraAngle;
  return -tilt;
}

bool CameraControlEngine::cameraSwitchIsUseful(float yImageOther)
{
  const float yAbs = yImageOther + theCameraInfo.opticalCenter.y;
  if(theImageInfo.fromLowerCamera())
    return yAbs < theCameraInfo.resolutionHeight;
  else
    return yAbs >= 0.0f;
}

float CameraControlEngine::calculateClippedAngle(const float yClipped, bool lowerCamera)
{
  Vector3<> clippedVector(theCameraInfo.focalLength, theCameraInfo.opticalCenter.x, theCameraInfo.opticalCenter.y - yClipped);
  clippedVector = theRobotCameraMatrix * clippedVector;
  const float clippedAngle = calculateTiltAngles(clippedVector, lowerCamera);
  DEBUG_RESPONSE("module:CameraControlEngine:getClippedTiltAngle",
  {
    OUTPUT(idText, text, "Clipped tilt angle is: " << clippedAngle);
  });
  return clippedAngle;
}

void CameraControlEngine::recordMinimumTiltAngles(bool isImageFromLowerCamera)
{
  const int yCenter = static_cast<int>(theCameraInfo.opticalCenter.y);
  int yClipped = yCenter;
  theBodyContour.clipBottom(static_cast<int>(theCameraInfo.opticalCenter.x), yClipped);
  CIRCLE("module:CameraControlEngine:yOfClipped", theCameraInfo.opticalCenter.x, yClipped, 3, 1, Drawings::ps_solid, ColorClasses::red, Drawings::ps_solid, ColorClasses::blue);
  const int camIdx = isImageFromLowerCamera ? 0 : 1;
  int panDegrees = static_cast<int>(abs(toDegrees(theFilteredJointData.angles[JointData::HeadYaw])));
  if(panDegrees > 119)
    panDegrees = 119;
  if(yClipped != yCenter || !isImageFromLowerCamera)
  {
    const float& angle = minimumTiltAngle[camIdx][panDegrees];
    const float& minUsefulTilt = calculateClippedAngle(static_cast<float>(yClipped), isImageFromLowerCamera);
    if(angle <= -0.9 || minUsefulTilt < angle)
      minimumTiltAngle[camIdx][panDegrees] = minUsefulTilt;
  }
  DEBUG_RESPONSE("module:CameraControlEngine:recordMinimumDisances",
  {
    Vector2<> pointOnField;
    Geometry::calculatePointOnField((int)(theCameraInfo.opticalCenter.x), yClipped, theCameraMatrix, theCameraInfo, pointOnField);
    const float distance = pointOnField.abs();
    float& currentMinDist = minimumDistances[camIdx][panDegrees];
    if(currentMinDist < 0.f || distance < currentMinDist)
      currentMinDist = distance;
  });
}

void CameraControlEngine::update(HeadAngleRequest& headAngleRequest)
{
  DECLARE_DEBUG_DRAWING("module:CameraControlEngine:panTiltTarget", "drawingOnField");
  DECLARE_DEBUG_DRAWING("module:CameraControlEngine:yOfFakeTarget", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:CameraControlEngine:yOfClipped", "drawingOnImage");
  MODIFY("parameters:CameraControlEngine", p);
  if(p.disableUpperCamera && p.disableLowerCamera)
  {
    OUTPUT(idText, text, "CameraControlEngine: Both cameras disabled. Enabling at lower camera.");
    p.disableLowerCamera = false;
  }

  bool isImageFromLowerCamera = theImageInfo.fromLowerCamera();

  DEBUG_RESPONSE("module:CameraControlEngine:saveMinimunAngle",
  {
    OutBinaryFile stream("minimumTiltAngle.tab");
    stream.write(minimumTiltAngle, sizeof(minimumTiltAngle));
  });
  DEBUG_RESPONSE("module:CameraControlEngine:saveMinimumDistances",
  {
    OutBinaryFile stream("minimumDistances.tab");
    stream.write("minimumDistances", sizeof(minimumDistances));
  });

  // Compute y image coordinate
  float yImage;
  float yImageOther;
  Vector3<> hip2Target; // Target relative to center of hip.
  if(theHeadMotionRequest.mode == HeadMotionRequest::panTiltMode)
  {
    /* pan-tilt-mode assumes that the lower camera is enabled, so the tilt angle offset between the head tilt joint
     * and the camera is already incorporated in the requested tilt angle. */
    const float tiltCurrent = theFilteredJointData.angles[JointData::HeadPitch] + theRobotDimensions.getHeadTiltToCameraTilt(!isImageFromLowerCamera);
    const float tiltOther = theFilteredJointData.angles[JointData::HeadPitch] + theRobotDimensions.getHeadTiltToCameraTilt(isImageFromLowerCamera);
    const float deltaTiltCurrent = theHeadMotionRequest.tilt - tiltCurrent;
    const float deltaTiltOther = theHeadMotionRequest.tilt - tiltOther;
    yImage = -tan(deltaTiltCurrent) * theCameraInfo.focalLength;
    yImageOther = -tan(deltaTiltOther) * theCameraInfo.focalLength;
  }
  else
  {
    if(theHeadMotionRequest.mode == HeadMotionRequest::targetMode)
      hip2Target = theHeadMotionRequest.target;
    else
      hip2Target = theTorsoMatrix.invert() * theHeadMotionRequest.target;
    calculateYImageCoordinate(hip2Target, theRobotCameraMatrix, theRobotCameraMatrixOther, yImage, yImageOther);
  }

  LINE("module:CameraControlEngine:yOfFakeTarget", 0, theCameraInfo.opticalCenter.y + yImage, theCameraInfo.resolutionWidth, theCameraInfo.opticalCenter.y + yImage, 1, Drawings::ps_solid, ColorRGBA(ColorClasses::blue));
  DEBUG_RESPONSE("module:CameraControlEngine:recordMinimumAngle",
  {
    recordMinimumTiltAngles(isImageFromLowerCamera);
  });

  // Calculate target angles.
  float targetTiltCurrent;
  float targetTiltOther;
  if(theHeadMotionRequest.mode == HeadMotionRequest::panTiltMode)
  {
    targetTiltCurrent = theHeadMotionRequest.tilt - theRobotDimensions.getHeadTiltToCameraTilt(!isImageFromLowerCamera);
    targetTiltOther = theHeadMotionRequest.tilt - theRobotDimensions.getHeadTiltToCameraTilt(isImageFromLowerCamera);
  }
  else
  {
    targetTiltCurrent = calculateTiltAngles(hip2Target, isImageFromLowerCamera);
    targetTiltOther = calculateTiltAngles(hip2Target, !isImageFromLowerCamera);
  }
  const float targetPan = theHeadMotionRequest.mode == HeadMotionRequest::panTiltMode ? theHeadMotionRequest.pan : atan2(hip2Target.y, hip2Target.x);
  int angleTableIdx = static_cast<int>(abs(toDegrees(targetPan)));
  // Angles >119 degrees aren't reachable.
  if(angleTableIdx > 119)
    angleTableIdx = 119;

  // Select camera
  if(p.disableLowerCamera)
  {
    requestedLowerCamera = false;
    lastFrameLowerClipped = false;
  }
  else if(p.disableUpperCamera)
  {
    requestedLowerCamera = true;
    lastFrameLowerClipped = false;
  }
  else if(theHeadMotionRequest.cameraControlMode == HeadMotionRequest::lowerCamera)
  {
    requestedLowerCamera = true;
    lastFrameLowerClipped = false;
  }
  else if(theHeadMotionRequest.cameraControlMode == HeadMotionRequest::upperCamera)
  {
    requestedLowerCamera = false; // The rest is only reached if cameraControlMode is autoCamera.
    lastFrameLowerClipped = false;
  }
  else if((isImageFromLowerCamera ? targetTiltCurrent : targetTiltOther) < (minimumTiltAngle[0][angleTableIdx]) + (lastFrameLowerClipped ? 0.1f : 0.0f))
  {
    requestedLowerCamera = false;
    lastFrameLowerClipped = true;
  }
  else if((isImageFromLowerCamera ? targetTiltOther : targetTiltCurrent) < minimumTiltAngle[1][angleTableIdx])
  {
    requestedLowerCamera = true;
    lastFrameLowerClipped = false;
  }
  else if(isImageFromLowerCamera && targetTiltOther > minimumTiltAngle[1][angleTableIdx] + 0.15)
  {
    requestedLowerCamera = false; // TODO: Remove keeper hack: Use upper camera by default.
    lastFrameLowerClipped = false;
  }

  // Set head angle request
  headAngleRequest.pan = targetPan;

  if(theHeadMotionRequest.mode == HeadMotionRequest::panTiltMode && theHeadMotionRequest.tilt == JointData::off)
    headAngleRequest.tilt = JointData::off;
  else if(isImageFromLowerCamera == requestedLowerCamera)
    headAngleRequest.tilt = targetTiltCurrent;
  else
    headAngleRequest.tilt = targetTiltOther;

  headAngleRequest.speed = theHeadMotionRequest.speed;
}

void CameraControlEngine::update(ImageRequest& imageRequest)
{
  DECLARE_PLOT("module:CameraControlEngine:imageReqest");
  PLOT("module:CameraControlEngine:imageReqest", requestedLowerCamera ? 2 : 1);

  imageRequest.requestedCamera = requestedLowerCamera ? ImageInfo::lowerCamera : ImageInfo::upperCamera;
}
