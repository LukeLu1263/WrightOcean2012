/**
* @file Controller/RemoteRobotWithPuppet.cpp
* Implementation of the base class of processes that communicate with a remote robot.
* @author <a href="mailto:Judith.Mueller@dfki.de">Judith Müller</a>
*/

#include <QVector>

#include "RemoteRobotWithPuppet.h"
#include "ConsoleRoboCupCtrl.h"
#include "RoboCupCtrl.h"

RemoteRobotWithPuppet::RemoteRobotWithPuppet(const char* name, const char* ip) :
  RobotConsole(theDebugReceiver, theDebugSender),
  theDebugReceiver(this, "Receiver.MessageQueue.O", false),
  theDebugSender(this, "Sender.MessageQueue.S", false),
  bytesTransfered(0),
  transferSpeed(0),
  timeStamp(0),
  me(0)
{
  strcpy(this->name, name);
  strcpy(this->ip, ip);
  mode = SystemCall::remoteRobotWithPuppet;

  // try to connect for one second
  Thread<RemoteRobotWithPuppet>::start(this, &RemoteRobotWithPuppet::connect);
  Thread<RemoteRobotWithPuppet>::stop();

  // get the robot
  VERIFY(me = (SimRobotCore2::Object*)RoboCupCtrl::application->resolveObject(QString("RoboCup.puppets.") + name, SimRobotCore2::object));

  QVector<QString> parts;
  parts.resize(1);
  QString position(".position");
  for(int i = 0; i < JointData::numOfJoints; ++i)
  {
    parts[0] = JointData::getName(JointData::Joint(i)) + position;
    jointActuators[i] = (SimRobotCore2::Actuator*)RoboCupCtrl::application->resolveObject(parts, me, SimRobotCore2::actuator);
  }
}

void RemoteRobotWithPuppet::connect()
{
  TcpConnection::connect(*ip ? ip : 0, 0xA1BD, TcpConnection::sender);
}

void RemoteRobotWithPuppet::run()
{
  while(isRunning())
    SystemCall::sleep(processMain());
}

int RemoteRobotWithPuppet::main()
{
  unsigned char* sendData = 0,
               * receivedData;
  int sendSize = 0,
      receivedSize = 0;
  MessageQueue temp;

  {
    // If there is something to send, prepare a package
    if(!theDebugSender.isEmpty())
    {
      SYNC;
      OutBinarySize size;
      size << theDebugSender;
      sendSize = size.getSize();
      sendData = new unsigned char[sendSize];
      OutBinaryMemory stream(sendData);
      stream << theDebugSender;
      // make backup
      theDebugSender.moveAllMessages(temp);
    }
  }

  // exchange data with the router
  if(!sendAndReceive(sendData, sendSize, receivedData, receivedSize) && sendSize)
  {
    // sending failed, restore theDebugSender
    SYNC;
    // move all messages since cleared (if any)
    theDebugSender.moveAllMessages(temp);
    // restore
    temp.moveAllMessages(theDebugSender);
  }

  // If a package was prepared, remove it
  if(sendSize)
  {
    delete [] sendData;
    sendSize = 0;
  }

  // If a package was received from the router program, add it to receiver queue
  if(receivedSize > 0)
  {
    SYNC;
    InBinaryMemory stream(receivedData, receivedSize);
    stream >> theDebugReceiver;

    delete [] receivedData;
  }

  return receivedSize > 0 ? 1 : 20;
}

void RemoteRobotWithPuppet::announceStop()
{
  {
    SYNC;
    debugOut.out.bin << DebugRequest("disableAll");
    debugOut.out.finishMessage(idDebugRequest);
  }
  SystemCall::sleep(1000);
  Thread<RemoteRobotWithPuppet>::announceStop();
}

void RemoteRobotWithPuppet::update()
{
  RobotConsole::update();

  if(SystemCall::getTimeSince(timeStamp) >= 2000)
  {
    int bytes = this->getOverallBytesSent() + this->getOverallBytesReceived() - bytesTransfered;
    bytesTransfered += bytes;
    timeStamp = SystemCall::getCurrentSystemTime();
    transferSpeed = bytes / 2000.0;
  }

  char buf[33];
  sprintf(buf, "%.1lf kb/s", transferSpeed);
  std::string statusText = std::string(robotName.mid(robotName.lastIndexOf(".") + 1).toAscii().constData()) + ": connected to " +
                           ip + ", " + buf;

  if(logPlayer.getNumberOfMessages() != 0)
  {
    sprintf(buf, "%u", logPlayer.numberOfFrames);
    statusText += std::string(", recorded ") + buf;
  }

  if(pollingFor)
  {
    statusText += statusText != "" ? ", polling for " : "polling for ";
    statusText += pollingFor;
  }


  ASSERT(me);

  //You need to put here more things the puppet should do, like fieldposition
  for(int i = 0; i < JointData::numOfJoints; ++i)
  {
    // Set angles
    const double& targetAngle(jointData.angles[i]);
    if(targetAngle != JointData::off &&
       targetAngle != JointData::ignore &&
       jointActuators[i]) // if joint does exist
      jointActuators[i]->setValue((targetAngle + jointCalibration.joints[i].offset) * jointCalibration.joints[i].sign);
  }

  if(moveOp != noMove)
  {
    Vector3<> position = movePos * 0.001f;
    if(moveOp == moveBoth)
    {
      RotationMatrix rotation(moveRot * (pi / 180));
      float rotation2[9];
      for(int i = 0; i < 3; ++i)
        for(int j = 0; j < 3; ++j)
          rotation2[j + i * 3] = rotation[i][j];
      me->move(&position.x, &rotation2[0]);
    }
    else if(moveOp == movePosition)
      me->move(&position.x);
    moveOp = noMove;
  }

  ((ConsoleRoboCupCtrl*)ConsoleRoboCupCtrl::controller)->printStatusText(statusText);
}


