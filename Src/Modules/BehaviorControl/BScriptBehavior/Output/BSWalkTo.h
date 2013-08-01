/**
* @file BSWalkTo.h
* Declaration of the walk_to basic behavior
* @author Colin Graf
*/

#pragma once

#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/BallHypotheses.h"
#include "Representations/Modeling/RobotsModel.h"
#include "Representations/Modeling/ObstacleModel.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "../InputRepresentations.h"

/**
* @class BSWalkTo
* The walk_to basic behavior
*/
class BSWalkTo
{
public:
  BSWalkTo(const InputRepresentations& inputRepresentations,
           MotionRequest* motionRequest);

  void walkTo(Pose2D target, float speed, bool rough);

private:
  struct PossibleAngle
  {
    float angle;
    float rating;
    int obstacleIndex;
  };

  static int comparePossibleAngle(const PossibleAngle* a, const PossibleAngle* b);

  /**
  * A collection of parameters for the robot pose validator.
  */
  class Parameters : public Streamable
  {
  public:
    /**
    * Default constructor.
    */
    Parameters()
      : obstacleAvoidanceDistance(800.f),
        obstacleAvoidanceMinRadius(300.f),
        obstacleAvoidanceMaxRadius(1000.f),
        visionObstacleTimeout(5000),
        ballAvoidanceDistance(300.f),
        ballAvoidanceMinRadius(300.f),
        ballAvoidanceMaxRadius(600.f),
        ballTimeout(500),
        penaltyAreaExpansion(0.f),
        penaltyAreaAvoidanceDistance(400.f),
        penaltyAreaAvoidanceMinRadius(300.f),
        penaltyAreaAvoidanceMaxRadius(1000.f),
        fieldBorderExpansion(300.f),
        fieldBorderAvoidanceDistance(400.f),
        fieldBorderAvoidanceMinRadius(300.f),
        fieldBorderAvoidanceMaxRadius(1000.f),
        goalPostAvoidanceDistance(400.f),
        goalPostAvoidanceMinRadius(300.f),
        goalPostAvoidanceMaxRadius(1000.f)
    {}

    float obstacleAvoidanceDistance;
    float obstacleAvoidanceMinRadius;
    float obstacleAvoidanceMaxRadius;
    int visionObstacleTimeout;
    float ballAvoidanceDistance;
    float ballAvoidanceMinRadius;
    float ballAvoidanceMaxRadius;
    int ballTimeout;
    float penaltyAreaExpansion;
    float penaltyAreaAvoidanceDistance;
    float penaltyAreaAvoidanceMinRadius;
    float penaltyAreaAvoidanceMaxRadius;
    float fieldBorderExpansion;
    float fieldBorderAvoidanceDistance;
    float fieldBorderAvoidanceMinRadius;
    float fieldBorderAvoidanceMaxRadius;
    float goalPostAvoidanceDistance;
    float goalPostAvoidanceMinRadius;
    float goalPostAvoidanceMaxRadius;

  private:
    /**
    * The method makes the object streamable.
    * @param in The stream from which the object is read.
    * @param out The stream to which the object is written.
    */
    virtual void serialize(In* in, Out* out)
    {
      STREAM_REGISTER_BEGIN();
      STREAM(obstacleAvoidanceDistance);
      STREAM(obstacleAvoidanceMinRadius);
      STREAM(obstacleAvoidanceMaxRadius);
      STREAM(visionObstacleTimeout);
      STREAM(ballAvoidanceDistance);
      STREAM(ballAvoidanceMinRadius);
      STREAM(ballAvoidanceMaxRadius);
      STREAM(ballTimeout);
      STREAM(penaltyAreaExpansion);
      STREAM(penaltyAreaAvoidanceDistance);
      STREAM(penaltyAreaAvoidanceMinRadius);
      STREAM(penaltyAreaAvoidanceMaxRadius);
      STREAM(fieldBorderExpansion);
      STREAM(fieldBorderAvoidanceDistance);
      STREAM(fieldBorderAvoidanceMinRadius);
      STREAM(fieldBorderAvoidanceMaxRadius);
      STREAM(goalPostAvoidanceDistance);
      STREAM(goalPostAvoidanceMinRadius);
      STREAM(goalPostAvoidanceMaxRadius);
      STREAM_REGISTER_FINISH();
    }
  };

  class Obstacle
  {
  public:
    Vector2<> leftPosition;
    Vector2<> rightPosition;
    Vector2<> centerPosition;
    float leftAngle;
    float rightAngle;
    float centerAngle;
    float openingAngle;
    bool active;
    bool leftIsValid;
    bool rightIsValid;

    Obstacle() {}
    Obstacle(const Vector2<>& leftPosition,
             const Vector2<>& rightPosition,
             const Vector2<>& centerPosition,
             float leftAngle,
             float rightAngle,
             bool leftIsValid = true,
             bool rightIsValid = true)
      : leftPosition(leftPosition),
        rightPosition(rightPosition),
        centerPosition(centerPosition),
        leftAngle(leftAngle),
        rightAngle(rightAngle),
        active(true),
        leftIsValid(leftIsValid),
        rightIsValid(rightIsValid)
    {
      if(rightAngle > leftAngle)
        rightAngle -= pi2;
      centerAngle = normalize((leftAngle + rightAngle) * 0.5f);
      openingAngle = leftAngle - rightAngle;
    }
  };

  Parameters p;

  MotionRequest* motionRequest;

  const FieldDimensions& theFieldDimensions;
  const RobotPose& theRobotPose;
  const RobotInfo& theRobotInfo;
  const GameInfo& theGameInfo;
  const ObstacleModel& theObstacleModel;
  const RobotsModel& theRobotsModel;
  const FrameInfo& theFrameInfo;
  const BallModel& theBallModel;
  const BallHypotheses& theBallHypotheses;

  Vector2<> leftPenCorner;
  Vector2<> leftPenGroundline;
  Vector2<> rightPenCorner;
  Vector2<> rightPenGroundline;
  Vector2<> leftOppCorner;
  Vector2<> rightOppCorner;
  Vector2<> leftOwnCorner;
  Vector2<> rightOwnCorner;
  Vector2<> goalPosts[4];

  unsigned int obstacleCount;
  Obstacle obstacles[16];

  float lastAvoidanceAngleOffset;

  void update();

  void addObstacle2(const Vector2<>& pos,
                    float expandLength,
                    float avoidanceMinRadius,
                    float avoidanceMaxRadius,
                    float avoidanceDistance);

  //void addObstacleWithMaxFactor(const Vector2<>& pos,
  //                              float expandLength,
  //                              float avoidanceMinRadius,
  //                              float avoidanceMaxRadius,
  //                              float maxFactor);

  void addObstacle2(const Vector2<>& left,
                    const Vector2<>& right,
                    const Vector2<>& center,
                    float obstacleDistance,
                    float avoidanceMinRadius,
                    float avoidanceMaxRadius,
                    float avoidanceDistance);

  void generateMotionRequest(const Pose2D& target, float speed);

  float getOrthogonalProjectionFactor(const Vector2<>& base,
                                      const Vector2<>& dir,
                                      const Vector2<>& point) const;
  float getSqrDistanceToLine(const Vector2<>& base,
                             const Vector2<>& dir,
                             float length,
                             const Vector2<>& point,
                             Vector2<>& orthogonalProjection) const;
};

