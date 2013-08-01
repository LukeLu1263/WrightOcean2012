/**
* @file ArmContactModelProvider.h
*
* Declaration of class ArmContactModelProvider.
* @author <a href="mailto:fynn@informatik.uni-bremen.de">Fynn Feldpausch</a>
*/

#pragma once

#include "Tools/RingBuffer.h"
#include "Tools/Module/Module.h"
#include "Representations/Modeling/FallDownState.h"
#include "Representations/Modeling/ArmContactModel.h"
#include "Representations/Infrastructure/JointData.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/MotionControl/MotionInfo.h"

MODULE(ArmContactModelProvider)
  REQUIRES(FilteredJointData)
  REQUIRES(MotionInfo)
  REQUIRES(JointRequest)
  REQUIRES(FrameInfo)
  REQUIRES(FallDownState)
  PROVIDES_WITH_MODIFY(ArmContactModel)
END_MODULE


/**
* @class ArmContactModelProvider
*/
class ArmContactModelProvider: public ArmContactModelProviderBase
{
public:
  /** Constructor */
  ArmContactModelProvider();

private:
  /**
  * A collection of parameters for this module.
  */
  class Parameters : public Streamable
  {
  public:
    /** Default constructor. */
    Parameters() {}

    float errorXThreshold;   /**< Maximum divergence of arm angleX (in degrees) that is not treated as an obstacle detection */
    float errorYThreshold;   /**< Maximum divergence of arm angleY (in degrees) that is not treated as an obstacle detection */
    float errorXDecrease;    /**< Decrease value of errorX per frame */
    float errorYDecrease;    /**< Decrease value of errorX per frame */
    bool debugMode;          /**< Enable debug mode */

  private:
    /**
    * The method makes the object streamable.
    * @param in The stream from which the object is read.
    * @param out The stream to which the object is written.
    */
    virtual void serialize(In* in, Out* out)
    {
      STREAM_REGISTER_BEGIN();
      STREAM(errorXThreshold);
      STREAM(errorYThreshold);
      STREAM(errorXDecrease);
      STREAM(errorYDecrease);
      STREAM(debugMode);
      STREAM_REGISTER_FINISH();
    }
  };

  struct ArmAngles
  {
    float leftX,             /**< X angle of left arm */
          leftY,             /**< Y angle of left arm */
          rightX,            /**< X angle of right arm */
          rightY;            /**< Y angle of right arm */
  };

  Parameters p;                              /**< The parameters of this module */
  RingBuffer<ArmAngles, 5> angleBuffer;      /**< Buffered arm angles to eliminate delay */
  float errorLeftX, errorLeftY,              /**< The accumulated angle errors of the left arm */
        errorRightX, errorRightY;            /**< The accumulated angle errors of the right arm */
  const unsigned int frameDelay;             /**< The size of the delay in frames */
  const int soundDelay;                      /**< Length of debug sound */
  unsigned int lastSoundTime;                /**< Time of last debug sound */

  /** Check shoulder joint angles for any collisions with other robots.
  * @param left Check the left or the right arm for collisions?
  * @return The contact state of the treated arm.
  */
  bool checkArm(bool left);

  /** Executes this module.
  * @param ArmContactModel The data structure that is filled by this module.
  */
  void update(ArmContactModel& ArmContactModel);

};
