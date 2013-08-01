/**
* @file Representations/BehaviorControl/HeadMotionProposal.h
* This file declares a class that represents a head motion proposal from the behavior.
* @author <A href="mailto:aseek@informatik.uni-bremen.de">Andreas Seekircher</A>
*/

#ifndef __HeadMotionProposal_H__
#define __HeadMotionProposal_H__

#include "Tools/Streams/Streamable.h"
#include "Representations/MotionControl/HeadMotionRequest.h"

/**
* @class HeadMotionProposal
* A class that represents the head motion proposal.
*/
class HeadMotionProposal : public HeadMotionRequest
{
private:
  virtual void serialize( In* in, Out* out)
  {  
    STREAM_REGISTER_BEGIN( );
    STREAM_BASE(HeadMotionRequest)
    STREAM(useActiveVision);
    STREAM(ballFactor);
    STREAM_REGISTER_FINISH();
  }

public:
  /**
  * Default constructor.
  */
  HeadMotionProposal() : useActiveVision(false), ballFactor(0) {}

  //if true, the head motion is overwritten by the active vision module
  bool useActiveVision;

  //the expected ball information is multiplied with ballFactor
  //ballFactor = 0 : only selflocalization
  //ballFactor > 0 : self- and ball localization,
  //                 value depends on the noise of BallLocator, the stdDevs in the SelfLocator...
  float ballFactor;
};

#endif // __HeadMotionProposal_H__
