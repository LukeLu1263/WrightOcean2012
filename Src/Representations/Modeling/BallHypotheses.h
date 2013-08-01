/**
* @file BallHypotheses.h
* Declaration of class BallHypotheses
* @author Colin Graf
*/

#pragma once

#include "Tools/Streams/Streamable.h"

/**
* @class BallHypotheses
* Encapsulates some hypotheses about the ball
*/
class BallHypotheses : public Streamable
{
public:
  unsigned int timeWhenDisappeared; /**< The time when the ball disappeared */

  /** Default constructor. */
  BallHypotheses() : timeWhenDisappeared(0) {}

private:
  /**
  * The method makes the object streamable
  * @param in The stream from which the object is read
  * @param out The stream to which the object is written
  */
  void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN();
    STREAM(timeWhenDisappeared);
    STREAM_REGISTER_FINISH();
  }
};
