/**
* @file KickInfo.h
* Declaration of a representation that contains information about the available kicks
* @author jeff
*/

#pragma once

#include "Representations/MotionControl/MotionRequest.h"
#include "Tools/Range.h"

/**
* @class KickInfo
* A representation that contains the nearest pose for shooting a goal
*/
class KickInfo : public Streamable
{
public:
  class Kick : public Streamable
  {
  public:
    float rotationOffset;
    Vector2<> ballOffset;
    float range;
    Range<> exclusionRange;
    unsigned int executionTime; /**< Time needed to perform the kick or extra penalty for deficient kicks (in ms) */
    MotionRequest::Motion motion;
    SpecialActionRequest::SpecialActionID specialAction;
    BikeRequest::BMotionID bikeMotionType;
    WalkRequest::KickType walkKickType;
    bool mirror;

    Kick() {}

    Kick(float rotationOffset, const Vector2<>& ballOffset, float range, unsigned int executionTime, MotionRequest::Motion motion, int kickId, bool mirror, const Range<>& exclusionRange = Range<>()) :
      rotationOffset(rotationOffset), ballOffset(ballOffset), range(range), exclusionRange(exclusionRange), executionTime(executionTime), motion(motion),
      specialAction(motion == MotionRequest::specialAction ? SpecialActionRequest::SpecialActionID(kickId) : SpecialActionRequest::numOfSpecialActionIDs),
      bikeMotionType(motion == MotionRequest::bike ? BikeRequest::BMotionID(kickId) : BikeRequest::numOfBMotionIDs),
      walkKickType(motion == MotionRequest::walk ? WalkRequest::KickType(kickId) : WalkRequest::none), mirror(mirror) {}

  private:
    void serialize(In* in, Out* out)
    {
      STREAM_REGISTER_BEGIN();
      STREAM(rotationOffset);
      STREAM(ballOffset);
      STREAM(range);
      STREAM(exclusionRange);
      STREAM(executionTime);
      STREAM(motion, MotionRequest);
      STREAM(specialAction, SpecialActionRequest);
      STREAM(bikeMotionType, BikeRequest);
      STREAM(walkKickType, WalkRequest);
      STREAM_REGISTER_FINISH();
    }
  };

  /**
  * Default constructor
  */
  KickInfo();

  enum KickType
  {
    bikeForward = 0,
    numOfKicks = 2,
  };

  Kick kicks[numOfKicks];

private:
  /**
  * Makes the object streamable
  * @param in The stream from which the object is read
  * @param out The stream to which the object is written
  */
  void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN();
    STREAM(kicks);
    STREAM_REGISTER_FINISH();
  }
};
