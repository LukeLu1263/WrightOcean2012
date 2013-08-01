/**
* @file RobotPoseValidator.h
* Declares a class that validates poses from the self locator.
* @author Colin Graf
*/

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/TeamInfo.h"
#include "Representations/Infrastructure/TeamMateData.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Configuration/DamageConfiguration.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/MotionControl/SpecialActionRequest.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/Perception/LinePercept.h"
#include "Representations/Perception/GoalPercept.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/FallDownState.h"
#include "Representations/Sensing/GroundContactState.h"
#include "Representations/MotionControl/OdometryData.h"
#include "Tools/Math/Matrix.h"
#include "Tools/RingBufferWithSum.h"

MODULE(RobotPoseValidator)
  REQUIRES(PotentialRobotPose)
  REQUIRES(CameraMatrix)
  REQUIRES(FieldDimensions)
  REQUIRES(DamageConfiguration)
  REQUIRES(CameraInfo)
  REQUIRES(LinePercept)
  REQUIRES(OwnTeamInfo)
  REQUIRES(MotionInfo)
  REQUIRES(OdometryData)
  REQUIRES(GoalPercept)
  REQUIRES(GroundContactState)
  REQUIRES(FallDownState)
  REQUIRES(FrameInfo)
  REQUIRES(TeamMateData) // for TEAM_OUTPUT_FAST
  PROVIDES_WITH_MODIFY_AND_OUTPUT_AND_DRAW(RobotPose)
  REQUIRES(RobotPose)
  PROVIDES_WITH_MODIFY(RobotPoseInfo)
END_MODULE

/**
* @class RobotPoseValidator
* A modules that determines the validity of a robot pose.
*/
class RobotPoseValidator : public RobotPoseValidatorBase
{
public:
  /**
  * Default constructor.
  */
  RobotPoseValidator();

private:
  /**
  * A collection of parameters for the robot pose validator.
  */
  class Parameters : public Streamable
  {
  public:
    /**
    * Default constructor.
    */
    Parameters() {}

    Pose2D respawnMaxPoseDistance; /**< The maximal admissible distance between the estimate from the self locator and the pose from the validator. */
    Pose2D respawnPoseDeviation; /**< The "deviation" of the pose from the self locator. (As long as it is near the correct pose.) */
    float lineRelationCorridor; /**< The corridor used for relating seen lines with field lines. */
    Pose2D odometryDeviation; /**< The percentage inaccuracy of the odometry. */
    Vector2<> odometryRotationDeviation; /**< A rotation deviation of each walked mm. */
    Pose2D filterProcessDeviation; /**< The process noise for estimating the robot pose. */
    Vector2<> robotRotationDeviation; /**< Deviation of the rotation of the robot's torso */
    Pose2D validationMaxDeviation; /**< The maximale admissible deviation of the robot pose. (Used for detecing valid poses.) */
    float validationMaxGoalPerceptDistance; /**< The maximale admissible distance of goal percepts to their expected position. (Used for detecing valid poses; In percentage of the distance to the goal post). */
    unsigned int validationMinGoalSightings; /**< The minimal required goal sightings used for detecting valid poses. */
    unsigned int validationMinUnknownGoalSightings; /**< The minimal required unknown goal sightings used for detecting valid poses. */

  private:
    /**
    * The method makes the object streamable.
    * @param in The stream from which the object is read.
    * @param out The stream to which the object is written.
    */
    virtual void serialize(In* in, Out* out)
    {
      STREAM_REGISTER_BEGIN();

      STREAM(respawnMaxPoseDistance);
      STREAM(respawnPoseDeviation);

      STREAM(lineRelationCorridor);

      STREAM(odometryDeviation);
      STREAM(odometryRotationDeviation);
      STREAM(filterProcessDeviation);
      STREAM(robotRotationDeviation);

      STREAM(validationMaxDeviation);
      STREAM(validationMaxGoalPerceptDistance);
      STREAM(validationMinGoalSightings);
      STREAM(validationMinUnknownGoalSightings);
      STREAM_REGISTER_FINISH();
    }
  };

  /**
  * A field line relative to the robot.
  */
  class FieldLine
  {
  public:
    Vector2<> start; /**< The starting point of the line. */
    Vector2<> end; /**< The ending point of the line. */
    Vector2<> dir; /**< The normalized direction of the line (from starting point). */
    float length; /**< The length of the line. */
    bool vertical; /**< Whether this is a vertical or horizontal line. */
  };

  /**
  * A line percept relative to the robot (in floats).
  */
  class LinePercept
  {
  public:
    Vector2<> start; /**< The starting point on field */
    Vector2<> end; /**< The ending point on field */
  };

  Parameters p; /**< A set of parameters for the module. */
  FieldLine fieldLines[24]; /**< Relevant field lines relative to the robot. */
  int countOfFieldLines; /**< Count of relevant field lines. */
  Vector2<> goalPosts[4]; /**< The positions of the goal posts. */

  Vector<2> translation; /**< The estimate of the translation. */
  Matrix<2, 2> translationCov; /**< The covariance matrix of the estimate. */
  float rotation; /**< The estimate of the rotation. */
  float rotationCov; /**< The variance of the estimate of the rotation. */
  Pose2D lastOdometryData; /**< OdometryData of the previous iteration. */

  unsigned int lastResetTime; /**< The time when the robot pose was reset. */
  bool validated; /**< Whether the pose is validated or not. */
  unsigned int validGoalSightingSinceLastReset; /**< Amount of matching goal post sightings since last loss of validity. */
  unsigned int validUnknownGoalSightingSinceLastReset; /**< Amout of "unknown" matching goal post sightings since last loss of validity. */

  /**
  * Initializes the module.
  */
  void init();

  /**
  * Updates the validated robot pose provided from this module.
  * @param robotPose The robotPose updated from the module
  */
  void update(RobotPose& robotPose);

  /**
  * Updates further information about the robot pose.
  * @param robotPose The robotPoseInfo updated from the module
  */
  void update(RobotPoseInfo& robotPoseInfo);

  bool intersectLineWithLine(const Vector2<>& lineBase1, const Vector2<>& lineDir1, const Vector2<>& lineBase2, const Vector2<>& lineDir2, Vector2<>& intersection) const;
  float getSqrDistanceToLine(const Vector2<>& base, const Vector2<>& dir, float length, const Vector2<>& point) const;
  float getSqrDistanceToLine(const Vector2<>& base, const Vector2<>& dir, const Vector2<>& point) const;
  Vector2<> getOrthogonalProjection(const Vector2<>& base, const Vector2<>& dir, const Vector2<>& point) const;

  void motionUpdate();
  void sensorUpdate(float angle, float x, float y, float angleVariance, float xOrYVariance);

  void useLinePercept(const LinePercept& linePercept, const FieldLine& fieldLine);
  void useCenterCircle(const Vector2<>& circlePos);
  void useGoalPost(const Vector2<>& postPos, const Vector2<>& postOnField);

  Matrix2x2<> getCovOfPointInWorld(const Vector2<>& pointInWorld, float pointZInWorld) const;
};
