/**
* @file Controller/RemoteRobotWithPuppet.h
* @author <a href="mailto:Judith.Mueller@dfki.de">Judith Müller</a>
*/

#pragma once

#include "RobotConsole.h"
#include "Tools/Debugging/TcpConnection.h"
#include "Representations/Configuration/JointCalibration.h"
#include "Representations/Infrastructure/JointData.h"
#include "SimRobotCore2.h"

class RoboCupCtrl;

/**
* @class Controller/RemoteRobotWithPuppet.h
* A class representing a process that communicates with a remote robot via TCP.
*/
class RemoteRobotWithPuppet : public RobotConsole, public TcpConnection, private Thread<RemoteRobotWithPuppet>
{
private:
  DEBUGGING;

  char name[80]; /**< The name of the robot. */
  char ip[80]; /**< The ip of the robot. */
  int bytesTransfered; /**< The number of bytes transfered so far. */
  double transferSpeed; /**< The transfer speed in kb/s. */
  unsigned timeStamp; /**< The time when the transfer speed was measured. */
  SimRobotCore2::Object* me; /**< The simulated roboter object. */

  SimRobotCore2::Actuator* jointActuators[JointData::numOfJoints]; /**< The handles to the actuator ports of the joints. */

  /**
  * The main loop of the process.
  */
  void run();

  /**
  * The function connects to another process.
  */
  void connect();

  /**
  * The function is called from the framework once in every frame.
  */
  virtual int main();

public:
  /**
  * Constructor.
  * @param name The name of the robot.
  * @param ip The ip address of the robot.
  */
  RemoteRobotWithPuppet(const char* name, const char* ip);

  /**
   * Destructor.
   */
  ~RemoteRobotWithPuppet() {Thread<RemoteRobotWithPuppet>::stop();}

  /**
  * The function starts the process.
  */
  void start() {Thread<RemoteRobotWithPuppet>::start(this, &RemoteRobotWithPuppet::run);}

  /**
  * The function is called to announce the termination of the process.
  */
  void announceStop();

  /**
  * The function must be called to exchange data with SimRobot.
  * It sends the motor commands to SimRobot and acquires new sensor data.
  */
  void update();

  /**
  * The function returns the name of the robot.
  * @return The name.
  */
  const char* getName() const {return name;}
};
