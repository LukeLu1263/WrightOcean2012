/**
* @file BallModel.h
*
* Declaration of class BallModel
*
* @author <A href="mailto:timlaue@informatik.uni-bremen.de">Tim Laue</A>
*/

#pragma once

#include "Tools/Math/Geometry.h"
#include "Representations/Modeling/RobotPose.h"


/**
 * @class BallState
 *
 * Base class for ball position and velocity.
 */
class BallState : public Streamable
{
  /** Streaming (with specifications) */
  virtual void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN();
    STREAM(position);
    STREAM(velocity);
    STREAM_REGISTER_FINISH();
  }

public:

  Vector2<> position; /**< The position of the ball relative to the robot (in mm)*/
  Vector2<> velocity; /**< The velocity of the ball relative to the robot (in mm/s)*/

  inline float getAngle() const
  {
    return atan2(position.y, position.x);
  }

  inline float getDistance() const
  {
    return position.abs();
  }

  inline Vector2<> getPositionInFieldCoordinates(const RobotPose& rp) const
  {
    return Geometry::relative2FieldCoord(rp, position.x, position.y);
  }

  Vector2<> getVelocityInFieldCoordinates(const RobotPose& rp) const
  {
    float c(rp.getCos());
    float s(rp.getSin());
    return Vector2<>(velocity.x * c - velocity.y * s, velocity.x * s + velocity.y * c);
  }

  void setPositionAndVelocityInFieldCoordinates(const Vector2<>& positionOnField,
                                                const Vector2<>& velocityOnField,
                                                const RobotPose& rp)
  {
    position = Geometry::fieldCoord2Relative(rp, positionOnField);
    float c(rp.getCos());
    float s(rp.getSin());
    velocity = Vector2<>(velocityOnField.x * c + velocityOnField.y * s,
                         -velocityOnField.x * s + velocityOnField.y * c);
  }

};


/**
 * @class BallModel
 *
 * Contains all current knowledge about the ball.
 */
class BallModel : public Streamable
{
  /** Streaming (with specifications) */
  virtual void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN();
    STREAM(lastPerception);
    STREAM(estimate);
    STREAM(endPosition);
    STREAM(lastSeenEstimate);
    STREAM(timeWhenLastSeen);
    STREAM_REGISTER_FINISH();
  }

public:
  /** Constructor */
  BallModel() : timeWhenLastSeen(0) {}

  BallState lastPerception; /**< The last seen position of the ball */
  BallState estimate; /**< The state of the ball estimated from own observations;
      it is propagated even if the ball is not seen */
  BallState lastSeenEstimate; /**< The last seen estimate */
  Vector2<> endPosition;
  unsigned timeWhenLastSeen; /**< Time stamp, indicating what its name says*/

  /** Draws something*/
  void draw()
  {
    DECLARE_DEBUG_DRAWING("representation:BallModel", "drawingOnField"); // drawing of the ball model
    COMPLEX_DRAWING("representation:BallModel",
    {
      {
        Vector2<>& position(estimate.position);
        Vector2<>& velocity(estimate.velocity);
        CIRCLE("representation:BallModel",
               position.x,
               position.y,
               45,
               0, // pen width
               Drawings::ps_solid,
               ColorClasses::orange,
               Drawings::bs_solid,
               ColorClasses::orange);
        ARROW("representation:BallModel", position.x, position.y,
              position.x + velocity.x, position.y + velocity.y, 5, 1, ColorClasses::orange);
      }
      {
        Vector2<>& position(lastSeenEstimate.position);
        Vector2<>& velocity(lastSeenEstimate.velocity);
        CIRCLE("representation:BallModel",
               position.x,
               position.y,
               45,
               0, // pen width
               Drawings::ps_solid,
               ColorRGBA(0, 0, 0, 220),
               Drawings::bs_solid,
               ColorRGBA(255, 128, 128, 220));
        ARROW("representation:BallModel", position.x, position.y,
              position.x + velocity.x, position.y + velocity.y, 5, 1, ColorRGBA(255, 128, 0, 220));
      }
      {
        Vector2<>& position(endPosition);
        CIRCLE("representation:BallModel",
               position.x,
               position.y,
               45,
               0, // pen width
               Drawings::ps_solid,
               ColorClasses::black,
               Drawings::bs_solid,
               ColorRGBA(168, 25, 99, 220));
      }
    });
  }
};

class GroundTruthBallModel : public BallModel
{
public:
  /** Draws something*/
  void draw()
  {
    DECLARE_DEBUG_DRAWING("representation:GroundTruthBallModel", "drawingOnField"); // drawing of the ground truth ball model
    COMPLEX_DRAWING("representation:GroundTruthBallModel",
    {
      const Vector2<>& position(lastPerception.position);
      const Vector2<>& velocity(estimate.velocity);
      CIRCLE("representation:GroundTruthBallModel",
             position.x,
             position.y,
             45,
             0, // pen width
             Drawings::ps_solid,
             ColorRGBA(255, 128, 0, 192),
             Drawings::bs_solid,
             ColorRGBA(255, 128, 0, 192));
      ARROW("representation:GroundTruthBallModel", position.x, position.y,
            position.x + velocity.x, position.y + velocity.y, 5, 1, ColorRGBA(255, 128, 0, 192));
    });
  }
};

class BallStateCompressed : public Streamable
{
  virtual void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN();
    STREAM_COMPRESSED_POSITION(position);
    STREAM_COMPRESSED_POSITION(velocity); // TODO: We have to test if short is sufficient for velocities.
    STREAM_REGISTER_FINISH();
  }
public:
  // 16 Bytes -> 8 Bytes
  Vector2<> position;
  Vector2<> velocity;

  BallStateCompressed()
  {
  }

  BallStateCompressed(const BallState& ballState)
    : position(ballState.position),
      velocity(ballState.velocity)
  {
  }

  BallState unpack()
  {
    BallState ballState;
    ballState.position = position;
    ballState.velocity = velocity;
    return ballState;
  }
};

class BallModelCompressed : public Streamable
{
  virtual void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN();
    STREAM(lastPerception);
    STREAM(estimate);
    STREAM(lastSeenEstimate);
    STREAM_COMPRESSED_POSITION(endPosition);
    STREAM(timeWhenLastSeen);
    STREAM_REGISTER_FINISH();
  }
public:
  // 60 Bytes -> 32 Bytes
  BallStateCompressed lastPerception;
  BallStateCompressed estimate;
  BallStateCompressed lastSeenEstimate;
  Vector2<> endPosition;
  unsigned timeWhenLastSeen;

  BallModelCompressed()
  {
  }

  BallModelCompressed(const BallModel& ballModel)
    : lastPerception(ballModel.lastPerception),
      estimate(ballModel.estimate),
      lastSeenEstimate(ballModel.lastSeenEstimate),
      endPosition(ballModel.endPosition),
      timeWhenLastSeen(ballModel.timeWhenLastSeen)
  {
  }

  BallModel unpack()
  {
    BallModel ballModel;
    ballModel.lastPerception = lastPerception.unpack();
    ballModel.estimate = estimate.unpack();
    ballModel.lastSeenEstimate = lastSeenEstimate.unpack();
    ballModel.endPosition = endPosition;
    ballModel.timeWhenLastSeen = timeWhenLastSeen;
    return ballModel;
  }
};
