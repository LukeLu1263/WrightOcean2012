/**
 * @file RobotPose.h
 *
 * The file contains the definition of the class RobotPose.
 *
 * @author <A href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</A>
 */

#pragma once

#include "Tools/Math/Pose2D.h"
#include "Tools/Debugging/DebugDrawings.h"

/**
* @class RobotPose
* A Pose2D with validity.
*/
class RobotPose : public Pose2D
{
protected:
  virtual void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN();
    STREAM_BASE(Pose2D);
    STREAM(validity);
    STREAM(deviation);
    STREAM_REGISTER_FINISH();
  }

public:
  float validity;                   /**< The validity of the robot pose. (0 = invalid, 1 = perfect) */
  float deviation;                  /**< The deviation of the robot pose. */
  ColorRGBA ownTeamColorForDrawing;  /**< As the name says... */

  enum
  {
    unknownDeviation = 100000,
  };

  /** Constructor */
  RobotPose() : validity(0.0f), deviation(unknownDeviation) {}

  /** Assignment operator
  * @param other Another RobotPose
  * @return A reference to the object after the assignment
  */
  const RobotPose& operator=(const RobotPose& other)
  {
    (Pose2D&) *this = (const Pose2D&) other;
    validity = other.validity;
    deviation = other.deviation;
    ownTeamColorForDrawing = other.ownTeamColorForDrawing;
    return *this;
  }

  /** Cast Contructor.
  * @param otherPose A Pose2D object
  */
  RobotPose(const Pose2D& otherPose)
  {
    (Pose2D&) *this = otherPose;
    validity = 0;
    deviation = 100000.f;
  }

  /** Assignment operator for Pose2D objects
  * @param other A Pose2D object
  * @return A reference to the object after the assignment
  */
  const RobotPose& operator=(const Pose2D& other)
  {
    (Pose2D&) *this = other;
    //validity is not set
    return *this;
  }

  /** Draws the robot pose to the field view*/
  void draw();
};

class GroundTruthRobotPose : public RobotPose
{
public:
  unsigned timestamp;
  /** Draws the robot pose to the field view*/
  void draw();
};

class PotentialRobotPose : public RobotPose
{
public:
  /** Draws the robot pose to the field view*/
  void draw() {};
};

/**
* @class RobotPoseInfo
* Encapsulates further information about the robot pose
*/
class RobotPoseInfo : public Streamable
{
public:
  unsigned int timeLastPoseReset; /**< The time when the robot pose was reset. */

private:
  /**
  * Makes the object streamable
  * @param in The stream from which the object is read
  * @param out The stream to which the object is written
  */
  virtual void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN();
    STREAM(timeLastPoseReset);
    STREAM_REGISTER_FINISH();
  }
};

class RobotPoseCompressed : public Streamable
{
  virtual void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN();
    STREAM_COMPRESSED_POSITION(translation);
    STREAM_COMPRESSED_ANGLE(rotation);
    STREAM_COMPRESSED_NORMALIZED_FLOAT(validity); // normalized means it is in [0:1]
    STREAM(deviation);
    STREAM(teamRed);
    STREAM_REGISTER_FINISH();
  }
public: // 24 Bytes -> 11 Bytes
  Vector2<> translation;
  float rotation;
  float validity;
  float deviation;
  bool teamRed;

  RobotPoseCompressed()
  {
  }

  RobotPoseCompressed(const RobotPose& robotPose)
    : translation(robotPose.translation),
      rotation(robotPose.rotation),
      validity(robotPose.validity),
      deviation(robotPose.deviation),
      teamRed(robotPose.ownTeamColorForDrawing.r == 255)
  {
  }

  RobotPose unpack()
  {
    RobotPose robotPose;
    robotPose.translation = translation;
    robotPose.rotation = rotation;
    robotPose.validity = validity;
    robotPose.deviation = deviation;
    robotPose.ownTeamColorForDrawing = teamRed ? ColorRGBA(0, 0, 255) : ColorRGBA(255, 0, 0);
    return robotPose;
  }
};
