/**
* @file ArmContactModel.h
*
* Declaration of class ArmContactModel.
* @author <a href="mailto:fynn@informatik.uni-bremen.de">Fynn Feldpausch</a>
*/

#pragma once


/**
* @class ArmContactModel
*/
class ArmContactModel : public Streamable
{
public:
  bool contactLeft;          /** The contact state of the robot's left arm. */
  bool contactRight;         /** The contact state of the robot's right arm. */

private:
  /** Streaming function
  * @param in Object for streaming in the one direction
  * @param out Object for streaming in the other direction
  */
  virtual void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN();
    STREAM(contactLeft);
    STREAM(contactRight);
    STREAM_REGISTER_FINISH();
  }
};
