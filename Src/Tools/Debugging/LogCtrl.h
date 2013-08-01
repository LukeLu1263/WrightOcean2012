/**
* @file LogCtrl.h
* Declaration of class LogCtrl
*
* @author <a href="mailto:matroc@tzi.de">Max Trocha</a>
*/

#pragma once

#include "Tools/Global.h"
#include "Tools/Settings.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "LogQueue.h"
#include "Platform/BHAssert.h"
#include "DebugRequest.h"
#include "Tools/Enum.h"
#include <sstream>
#include <list>

class LogCtrl : public MessageHandler
{
public:
  ENUM(LogMode,
    DISABLED, // Don't log
    ENABLED, // Logging always on
    ON_GAMESTATES, // Logging enabled if gamestate is ready, set or play
    ON_GAMECONTROLLER // Like ON_GAMESTATES but only if gamecontroller is present
  );

  ENUM(LogState,
    INITIAL,
    WAIT,
    LOG,
    ENABLE_NEXT_FRAME,
    DISABLE_NEXT_FRAME,
    IGNORE_ALL
  );

  LogCtrl(MessageHandler* debug);
  ~LogCtrl();
  bool handleMessage(InMessage& message);
  void processBegin(char process);
  bool prepare(InMessage* message = 0);

private:
  class Configuration : public Streamable
  {
  private:
    void serialize(In* in, Out* out)
    {
      STREAM_REGISTER_BEGIN();
      STREAM(queueSize);
      STREAM(logFileName);
      STREAM(logMode, LogCtrl);
      STREAM(logMessages);
      STREAM(logCognition);
      STREAM(logMotion);
      STREAM_REGISTER_FINISH();
    }
  public:
    int queueSize;
    std::string logFileName;
    LogMode logMode;
    std::vector<std::string> logMessages;
    bool logCognition;
    bool logMotion;

    Configuration() :
      queueSize(4 << 20), //4mb
      logFileName("log.log"),
      logMode(DISABLED),
      logCognition(true),
      logMotion(false) {};
  } config;

  LogQueue* logQueue;
  MessageQueue tmpQueue;
  LogState logState;
  MessageHandler* debug;
  RoboCup::uint8 lastGameState;
  RoboCup::uint16 lastPenalty;
  float timeSinceLastPackageReceived;
  char currentProcess;
  void initQueue();
  void initInfoRequests();
  void debugRequest(std::string name, bool once = false);
};
