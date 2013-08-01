/**
* @file BoardInfo.h
* The file declares a class that represents information about the connection to
* all boards of the robot.
* @author <a href="mailto:Thomas.Röfer@dfki.de">Thomas Röfer</a>
*/

#pragma once

#include "Tools/Streams/Streamable.h"
#include "Tools/Enum.h"


/**
* @class BoardInfo
* A class that represents information about the connection to
* all boards of the robot.
*/
class BoardInfo : public Streamable
{
private:
  /**
  * The method makes the object streamable.
  * @param in The stream from which the object is read (if in != 0).
  * @param out The stream to which the object is written (if out != 0).
  */
  virtual void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN();
    STREAM(ack);
    STREAM(nack);
    STREAM(error);
    STREAM_REGISTER_FINISH();
  }

public:
  ENUM(Board,
    chestBoard,
    battery,
    usBoard,
    inertialSensor,
    headBoard,
    earLeds,
    faceBoard,
    leftShoulderBoard,
    leftArmBoard,
    rightShoulderBoard,
    rightArmBoard,
    leftHipBoard,
    leftThighBoard,
    leftShinBoard,
    leftFootBoard,
    rightHipBoard,
    rightThighBoard,
    rightShinBoard,
    rightFootBoard
  );

  int ack[numOfBoards], /**< The number of times a package has been successfully received for each board. */
      nack[numOfBoards], /**< The number of times an error occured during package reception for each board. */
      error[numOfBoards]; /**< The current error number for each board. */

  /** Default constructor */
  BoardInfo()
  {
    for(int i = 0; i < numOfBoards; ++i)
      ack[i] = nack[i] = error[i] = 0;
  }
};
