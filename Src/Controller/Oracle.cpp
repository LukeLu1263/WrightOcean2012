/**
 * @file Controller/Oracle.cpp
 * Implementation of class Oracle for SimRobotQt.
 * @author Colin Graf
 */

#include <QString>
#include <QVector>

#include "Oracle.h"
#include "Controller/RoboCupCtrl.h"
#include "Controller/TeamComm3DCtrl.h"
#include "Platform/SystemCall.h"
#include "Representations/Infrastructure/Image.h"
#include "Representations/Infrastructure/SensorData.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/MotionControl/OdometryData.h"
#include "Representations/Sensing/OrientationData.h"
#include "Representations/Perception/ImageInfo.h"
#include "Tools/Streams/InStreams.h"
#include "Tools/Settings.h"

SimRobotCore2::Sensor* Oracle::activeCameras[10];
unsigned Oracle::activeCameraCount = 0;

float Oracle::walkHeight = 0;
SimRobot::Object* Oracle::ball = 0;
unsigned int Oracle::lastKick = 0;
Pose2D Oracle::lastKickPose;
Pose2D Oracle::lastClosestPose;
float Oracle::minBallDistance = numeric_limits<float>::max();
FieldDimensions Oracle::fieldDimensions;

Oracle::Oracle() :
  blue(false), robot(0), leftFoot(0), rightFoot(0), lastTimeStamp(0), framesSinceCameraSwitching(0), activeCameraIndex(activeCameraCount++), lastUsMeasurement(0) {}

Oracle::~Oracle()
{
  --activeCameraCount;
}

void Oracle::init(SimRobot::Object* robot)
{
  ASSERT(this->robot == 0);
  this->robot = (SimRobotCore2::Object*)robot;
  application = RoboCupCtrl::application ? RoboCupCtrl::application : TeamComm3DCtrl::application;
  SimRobot::Object* simulation = application->resolveObject("Simulation2", SimRobotCore2::simulation);

  motionCycleTime = (float)((SimRobotCore2::Simulation2*)simulation)->getStepLength();

  // get the robot name
  const QString& fullName = robot->getFullName();
  blue = fullName[fullName.length() - 1] < '5';

  // get feet (for pose and odometry)
  QVector<QString> parts;
  parts.resize(1);
  parts[0] = "RFoot";
  VERIFY(rightFoot = (SimRobotCore2::Object*)application->resolveObject(parts, robot, SimRobotCore2::object));
  parts[0] = "LFoot";
  VERIFY(leftFoot = (SimRobotCore2::Object*)application->resolveObject(parts, robot, SimRobotCore2::object));

  // get joints
  parts.resize(1);
  QString position(".position");
  for(int i = 0; i < JointData::numOfJoints; ++i)
  {
    parts[0] = QString(JointData::getName(JointData::Joint(i))) + position;
    VERIFY(jointSensors[i] = (SimRobotCore2::Sensor*)application->resolveObject(parts, robot, SimRobotCore2::sensor));
    VERIFY(jointActuators[i] = (SimRobotCore2::Actuator*)application->resolveObject(parts, robot, SimRobotCore2::actuator));
  }

  // imu sensors
  parts.resize(1);
  parts[0] = "Gyroscope.angularVelocities";
  gyroSensor = application->resolveObject(parts, robot, SimRobotCore2::sensor);

  parts[0] = "Accelerometer.acceleration";
  accSensor = application->resolveObject(parts, robot, SimRobotCore2::sensor);

  // cameras
  parts[0] = "CameraTop.image";
  upperCameraSensor = application->resolveObject(parts, robot, SimRobotCore2::sensor);
  parts[0] = "CameraBottom.image";
  lowerCameraSensor = application->resolveObject(parts, robot, SimRobotCore2::sensor);
  cameraSensor = lowerCameraSensor;
  activeCameras[activeCameraIndex] = (SimRobotCore2::Sensor*)cameraSensor;

  // sonars
  parts[0] = "SonarLeft.distance";
  leftUsSensor = (SimRobotCore2::Sensor*)application->resolveObject(parts, robot, SimRobotCore2::sensor);
  parts[0] = "SonarRight.distance";
  rightUsSensor = (SimRobotCore2::Sensor*)application->resolveObject(parts, robot, SimRobotCore2::sensor);
  parts[0] = "SonarCenterLeft.distance";
  centerLeftUsSensor = (SimRobotCore2::Sensor*)application->resolveObject(parts, robot, SimRobotCore2::sensor);
  parts[0] = "SonarCenterRight.distance";
  centerRightUsSensor = (SimRobotCore2::Sensor*)application->resolveObject(parts, robot, SimRobotCore2::sensor);

  walkHeight = 60.f;

  // load calibration
  InConfigMap stream(Global::getSettings().expandRobotFilename("jointCalibration.cfg"));
  ASSERT(stream.exists());
  stream >> jointCalibration;
}

Oracle::BallOut Oracle::updateBall()
{
  if(!ball)
    return NONE;

  BallOut result = NONE;
  float ballPosZ;
  Vector2<> ballPos = getPosition(ball, ballPosZ);
  if(!fieldDimensions.isInsideField(ballPos))
  {
    if(fabs(ballPos.y) < fieldDimensions.yPosLeftGoal) // goal
    {
      result = ballPos.x > fieldDimensions.xPosOpponentGroundline ? GOAL_BY_RED : GOAL_BY_BLUE;
    }
    else
    {
      Pose2D kickerPose;
      if(SystemCall::getTimeSince(lastKick) < 10000)
        kickerPose = lastKickPose; // kicked out
      else
        kickerPose = lastClosestPose; // dribbled out

      float x;
      if((Pose2D(ballPos) - kickerPose).translation.x > 0) // 1m behind robot
        x = (kickerPose + Pose2D(-1000, 0)).translation.x;
      else // 1m behind where ball went out
        x = (Pose2D(kickerPose.rotation, ballPos) + Pose2D(-1000, 0)).translation.x;

      if(fabs(ballPos.x) > fieldDimensions.xPosOpponentGroundline && (Pose2D(x, 0) - Pose2D(kickerPose.rotation)).translation.x > 0)
        x = 0; // center line
      else if(x < -2000)
        x = -2000; // clip
      else if(x > 2000)
        x = 2000; // clip
      ballPos.x = x;

      if(ballPos.y < 0)
        ballPos.y = -1600; // right throw-in line
      else
        ballPos.y = 1600; // left throw-in line

      moveBall(Vector3<>(ballPos.x, ballPos.y, ballPosZ), true);
      result = kickerPose.rotation == 0 ? OUT_BY_RED : OUT_BY_BLUE;
    }
  }
  minBallDistance = numeric_limits<float>::max();

  return result;
}

void Oracle::setBall(SimRobot::Object* ball)
{
  Oracle::ball = ball;
  lastKick = 0;
  if(ball)
    fieldDimensions.load();
}

void Oracle::getRobotPose(RobotPose& robotPose) const
{
  ASSERT(rightFoot && leftFoot);

  Pose2D leftPose, rightPose;
  float leftPoseZ, rightPoseZ, z;
  getPose2D(leftFoot, leftPose, leftPoseZ);
  getPose2D(rightFoot, rightPose, rightPoseZ);
  getPose2D(robot, (Pose2D&)robotPose, z);
  robotPose.translation = (leftPose.translation + rightPose.translation) * 0.5f;

  if(leftPoseZ > walkHeight || rightPoseZ > walkHeight)
  {
    lastKick = SystemCall::getCurrentSystemTime();
    lastKickPose = Pose2D(blue ? pi : 0, robotPose.translation);
  }

  if(ball)
  {
    float z;
    float ballSqrDistance = (robotPose.translation - getPosition(ball, z)).squareAbs();
    if(ballSqrDistance < minBallDistance * minBallDistance)
    {
      lastClosestPose = Pose2D(blue ? pi : 0, robotPose.translation);
      minBallDistance = sqrt(ballSqrDistance);
    }
  }

  if(blue)
    robotPose = Pose2D(pi) + robotPose;

  robotPose.validity = 1.f;
  robotPose.deviation = 1.f;
}

void Oracle::getOdometryData(const RobotPose& robotPose, OdometryData& odometryData) const
{
  ASSERT(robot);
  (Pose2D&)odometryData = blue ? (Pose2D(pi) + robotPose) : (const Pose2D&)robotPose;
}

void Oracle::getBallModel(const RobotPose& robotPose, BallModel& ballModel)
{
  ASSERT(robot);
  if(ball)
  {
    unsigned int timeWhenLastSeen = SystemCall::getCurrentSystemTime();
    Pose2D pose;
    float z;
    getPose2D(ball, pose, z);
    if(blue)
      pose.translation = -pose.translation;
    Vector2<float> velocity((pose.translation - lastBallPosition) / float(timeWhenLastSeen - ballModel.timeWhenLastSeen) * 1000.0f);
    ballModel.lastPerception.setPositionAndVelocityInFieldCoordinates(pose.translation, velocity, robotPose);
    ballModel.estimate = ballModel.lastPerception;
    ballModel.lastSeenEstimate = ballModel.lastPerception;
    ballModel.timeWhenLastSeen = timeWhenLastSeen;
    lastBallPosition = pose.translation;
    if(velocity != Vector2<>())
      ballModel.endPosition = ballModel.estimate.position + Vector2<>(ballModel.estimate.velocity).normalize(ballModel.estimate.velocity.squareAbs() / (2.f * (float) fieldDimensions.ballFriction));
    else
      ballModel.endPosition = ballModel.estimate.position;
  }
}

void Oracle::getImage(Image& image)
{
  ASSERT(robot);

  if(++framesSinceCameraSwitching <= 3)
    return;

  if(cameraSensor)
  {
    ((SimRobotCore2::Sensor*)cameraSensor)->renderCameraImages(activeCameras, activeCameraCount);

    ASSERT(!image.isReference);
    const int w = image.cameraInfo.resolutionWidth;
    const int h = image.cameraInfo.resolutionHeight;

    const int w3 = w * 3, w2 = w * 2;
    unsigned char* src = (unsigned char*)((SimRobotCore2::Sensor*)cameraSensor)->getValue().byteArray;
    unsigned char* srcLineEnd = src;
    Image::Pixel* destBegin = (Image::Pixel*)image.image;
    Image::Pixel* dest;
    int r1, g1, b1, yy, cr;
    for(int y = h - 1; y >= 0; --y)
    {
      for(srcLineEnd += w3, dest = destBegin + y * w2; src < srcLineEnd;)
        for(int i = 0; i < 4; ++i)
        {
          yy = 306 * (r1 = *(src++));
          cr = 130560 - 429 * (g1 = *(src++)) + 512 * r1;
          dest->cb = (unsigned char)((130560 - 173 * r1 - 339 * g1 + 512 * (b1 = *(src++))) >> 10);
          yy += 117 * b1 + 601 * g1;
          cr -= 83 * b1;
          dest->y = dest->yCbCrPadding = (unsigned char)(yy >> 10);
          (dest++)->cr = (unsigned char)(cr >> 10);
        }
    }
  }

  image.timeStamp = SystemCall::getCurrentSystemTime();
}

void Oracle::getAndSetJointData(const JointData& jointRequest, JointData& jointData) const
{
  ASSERT(robot);

  jointData.cycleTime = motionCycleTime;
  for(int i = 0; i < JointData::numOfJoints; ++i)
  {
    // Get angles
    if(jointSensors[i])
      jointData.angles[i] = float(((SimRobotCore2::Sensor*)jointSensors[i])->getValue().floatValue * jointCalibration.joints[i].sign - jointCalibration.joints[i].offset);

    // Set angles
    const float& targetAngle(jointRequest.angles[i]);
    if(targetAngle != JointData::off &&
       targetAngle != JointData::ignore &&
       jointActuators[i]) // if joint does exist
      ((SimRobotCore2::Actuator*)jointActuators[i])->setValue((targetAngle + jointCalibration.joints[i].offset) * jointCalibration.joints[i].sign);
  }
  jointData.timeStamp = SystemCall::getCurrentSystemTime();
}

void Oracle::setJointData(const JointData& jointRequest) const
{
  ASSERT(robot);
  for(int i = 0; i < JointData::numOfJoints; ++i)
  {
    // Set angles
    const float& targetAngle(jointRequest.angles[i]);
    if(targetAngle != JointData::off &&
       targetAngle != JointData::ignore &&
       jointActuators[i]) // if joint does exist
      ((SimRobotCore2::Actuator*)jointActuators[i])->setValue((targetAngle + jointCalibration.joints[i].offset) * jointCalibration.joints[i].sign);
  }
}

void Oracle::getAndSetCamera(const ImageRequest& request, ImageInfo& info)
{
  if(info.camera != request.requestedCamera)
    framesSinceCameraSwitching = 0;

  info.prevCamera = info.camera;
  cameraSensor = request.requestedCamera == ImageInfo::upperCamera ? upperCameraSensor : lowerCameraSensor;
  activeCameras[activeCameraIndex] = (SimRobotCore2::Sensor*)cameraSensor;
  info.camera = request.requestedCamera;
}

void Oracle::getSensorData(SensorData& sensorData)
{
  ASSERT(robot);

  sensorData.timeStamp = SystemCall::getCurrentSystemTime();

  // Gyro
  const float* floatArray = ((SimRobotCore2::Sensor*)gyroSensor)->getValue().floatArray;
  sensorData.data[SensorData::gyroX] = floatArray[0] + 12.f; // 12.f = bias
  sensorData.data[SensorData::gyroY] = floatArray[1] + 6.f; // 6.f = bias
  sensorData.data[SensorData::gyroZ] = 0.f; //float(doubleArray[2]); // nao style :(

  // Acc
  floatArray = ((SimRobotCore2::Sensor*)accSensor)->getValue().floatArray;
  sensorData.data[SensorData::accX] = floatArray[0] / -9.81f + 0.1f; // 0.1f = bias
  sensorData.data[SensorData::accY] = floatArray[1] / -9.81f + 0.2f; // 0.2f = bias
  sensorData.data[SensorData::accZ] = floatArray[2] / -9.81f + 0.05f; // 0.05f = bias

  // angle
  const float(*world2robot)[3] = (float(*)[3])((SimRobotCore2::Object*)robot)->getRotation();
  sensorData.data[SensorData::angleX] = float(atan2(world2robot[1][2], world2robot[2][2]));
  sensorData.data[SensorData::angleY] = -float(atan2(world2robot[0][2], world2robot[2][2]));

  // Battery
  sensorData.data[SensorData::batteryLevel] = 1.0f;

  // ultrasonic (model approximation. not absolutely correct in reality)
  static const float scale = 1000; //meter to millimeter
  if(sensorData.timeStamp - lastUsMeasurement >= 70 - 5)
  {
    SensorData::UsActuatorMode& usMode = sensorData.usActuatorMode;
    lastUsMeasurement = sensorData.timeStamp;
    sensorData.usTimeStamp = sensorData.timeStamp;
    // Determine next us actuator mode:
    // Current sensor sequence seems to be: rightToLeft => leftToLeft => leftToRight => rightToRight => ...
    if(usMode == SensorData::rightToLeft || usMode == SensorData::leftToRight)
      usMode = usMode == SensorData::rightToLeft ? SensorData::leftToLeft : SensorData::rightToRight;
    else if(usMode == SensorData::leftToLeft || usMode == SensorData::rightToRight)
      usMode = usMode == SensorData::leftToLeft ? SensorData::leftToRight : SensorData::rightToLeft;
    // Create sensor data that is compatible to current ObstacleModel ;-)
    const float centerLeftUsValue =  float(((SimRobotCore2::Sensor*)centerLeftUsSensor)->getValue().floatValue * scale);
    const float centerRightUsValue = float(((SimRobotCore2::Sensor*)centerRightUsSensor)->getValue().floatValue * scale);
    if(usMode == SensorData::rightToLeft || usMode == SensorData::leftToRight)
    {
      sensorData.data[SensorData::usR] = centerLeftUsValue;  //[!]
      sensorData.data[SensorData::usL] = centerRightUsValue; //[!]
    }
    else if(usMode == SensorData::leftToLeft || usMode == SensorData::rightToRight)
    {
      const float leftUsValue =        float(((SimRobotCore2::Sensor*)leftUsSensor)->getValue().floatValue * scale);
      const float rightUsValue =       float(((SimRobotCore2::Sensor*)rightUsSensor)->getValue().floatValue * scale);
      sensorData.data[SensorData::usL] = centerLeftUsValue < leftUsValue ? centerLeftUsValue : leftUsValue;
      sensorData.data[SensorData::usR] = centerRightUsValue < rightUsValue ? centerRightUsValue : rightUsValue;
    }
  }

  // TODO ...
}

void Oracle::getOrientationData(const SensorData& sensorData, OrientationData& orientationData)
{
  ASSERT(robot);
  getPose3D(robot, robotPose3D);
  const Pose3D offset(lastRobotPose3D.invert().conc(robotPose3D));
  orientationData.orientation.x = sensorData.data[SensorData::angleX];
  orientationData.orientation.y = sensorData.data[SensorData::angleY];
  float timeScale = 1.f / (float(sensorData.timeStamp - lastTimeStamp) * 0.001f);
  orientationData.velocity.x = float(offset.translation.y * timeScale);
  orientationData.velocity.y = -float(offset.translation.x * timeScale);
  orientationData.velocity.z = float(offset.translation.z * timeScale);
  lastRobotPose3D = robotPose3D;
  lastTimeStamp = sensorData.timeStamp;
}

void Oracle::moveRobot(const Vector3<>& pos, const Vector3<>& rot, bool changeRotation)
{
  ASSERT(robot);

  Vector3<> position = pos * 0.001f;
  if(changeRotation)
  {
    RotationMatrix rotation(rot);
    float rotation2[9];
    for(int i = 0; i < 3; ++i)
      for(int j = 0; j < 3; ++j)
        rotation2[j + i * 3] = rotation[i][j];
    ((SimRobotCore2::Object*)robot)->move(&position.x, rotation2);
  }
  else
    ((SimRobotCore2::Object*)robot)->move(&position.x);
}

void Oracle::moveBall(const Vector3<>& pos, bool resetDynamics)
{
  Vector3<> position = pos * 0.001f;
  ((SimRobotCore2::Object*)ball)->move(&position.x);
  if(resetDynamics)
    ((SimRobotCore2::Object*)ball)->resetDynamics();
}

Vector2<> Oracle::getPosition(SimRobot::Object* obj, float& z)
{
  const float* position = ((SimRobotCore2::Object*)obj)->getPosition();
  z = position[2] * 1000.f;
  return Vector2<>(position[0] * 1000.f, position[1] * 1000.f);
}

void Oracle::getPose2D(SimRobot::Object* obj, Pose2D& pose2D, float& z) const
{
  const float(*col)[3] = (float(*)[3])((SimRobotCore2::Object*)obj)->getRotation();

  // getAngleZ()
  float h = sqrt(col[0][0] * col[0][0] + col[0][1] * col[0][1]);
  if(h)
  {
    h = col[0][0] / h;
    if(h > 1)
      h = 1;
    else if(h < -1)
      h = -1;
    pose2D.rotation = acos(h) * (col[0][1] < 0 ? -1 : 1);
  }
  else
    pose2D.rotation = 0;

  const float* position = ((SimRobotCore2::Object*)obj)->getPosition();
  pose2D.translation.x = position[0] * 1000.f;
  pose2D.translation.y = position[1] * 1000.f;
  z = position[2] * 1000.f;
}

void Oracle::getPose3D(SimRobot::Object* obj, Pose3D& pose3D) const
{
  const float(*rotation)[3] = (float(*)[3])((SimRobotCore2::Object*)obj)->getRotation();
  const float* translation = ((SimRobotCore2::Object*)obj)->getPosition();

  pose3D.rotation.c0.x = rotation[0][0];
  pose3D.rotation.c0.y = rotation[0][1];
  pose3D.rotation.c0.z = rotation[0][2];
  pose3D.rotation.c1.x = rotation[1][0];
  pose3D.rotation.c1.y = rotation[1][1];
  pose3D.rotation.c1.z = rotation[1][2];
  pose3D.rotation.c2.x = rotation[2][0];
  pose3D.rotation.c2.y = rotation[2][1];
  pose3D.rotation.c2.z = rotation[2][2];
  pose3D.translation.x = translation[0];
  pose3D.translation.y = translation[1];
  pose3D.translation.z = translation[2];
  pose3D.translation *= 1000.;
}
