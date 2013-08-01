/**
* @file LogCtrl.cpp
* Implementation of class LogCtrl
*
*/

#define INTERRUPTION (lastPenalty != PENALTY_NONE\
                      || lastGameState == STATE_INITIAL\
                      || lastGameState == STATE_FINISHED\
                      ||(config.logMode == ON_GAMECONTROLLER\
                         && timeSinceLastPackageReceived > 10000.f))

#include "LogCtrl.h"
#include <iterator>

bool LogCtrl::handleMessage(InMessage& message)
{
  if(prepare(&message))
    return logQueue->handleMessage(message);
  return false;
}

LogCtrl::LogCtrl(MessageHandler* debug)
  : logQueue(0),
    logState(INITIAL),
    debug(debug),
    lastGameState(STATE_INITIAL),
    lastPenalty(PENALTY_NONE),
    timeSinceLastPackageReceived(999999.f),
    currentProcess('x')
{
  tmpQueue.setSize(128);
  InConfigMap file(Global::getSettings().expandLocationFilename("logging.cfg"));
  ASSERT(file.exists());
  file >> config;
}

LogCtrl::~LogCtrl()
{
  delete logQueue;
}


void LogCtrl::processBegin(char process)
{
  currentProcess = process;
  tmpQueue.out.bin << process;
  tmpQueue.out.finishMessage(idProcessBegin);
  tmpQueue.handleAllMessages(*this);
  tmpQueue.clear();
}

bool LogCtrl::prepare(InMessage* message)
{
  //check for gamestate/penalty/frame changes
  bool stateChange = false;
  bool penaltyChange = false;
  bool frameChange = false;
  if(message)
  {
    switch(message->getMessageID())
    {
    case idGameInfo:
    {
      GameInfo gi;
      message->bin >> gi;
      stateChange = lastGameState != gi.state;
      lastGameState = gi.state;
      timeSinceLastPackageReceived = gi.timeSinceLastPackageReceived;
      break;
    }
    case idRobotInfo:
    {
      RobotInfo ri;
      message->bin >> ri;
      penaltyChange = lastPenalty != ri.penalty;
      lastPenalty = ri.penalty;
      break;
    }
    case idProcessBegin:
    {
      frameChange = true;
      break;
    }
    default:
      ;
    }
  }

  switch(logState)
  {
  case INITIAL:
    switch(config.logMode)
    {
    case ENABLED:
      initQueue();
      logState = LOG;
      break;
    case DISABLED:
      logState = IGNORE_ALL;
      return false;
    case ON_GAMECONTROLLER:
    case ON_GAMESTATES:
      initInfoRequests();
      logState = WAIT;
      return false;
    default:
      break;
    }
    break;
  case WAIT:
    if(!INTERRUPTION)
      logState = ENABLE_NEXT_FRAME;
    return false;
  case LOG:
    if(INTERRUPTION && config.logMode != ENABLED)
      logState = DISABLE_NEXT_FRAME;
    break;
  case ENABLE_NEXT_FRAME:
    if(frameChange)
    {
      initQueue();
      logState = LOG;
      break;
    }
    return false;
  case DISABLE_NEXT_FRAME:
    if(frameChange)
    {
      if(logQueue != 0)
        logQueue->save();
      logState = WAIT;
      return false;
    }
    break;
  case IGNORE_ALL:
    return false;
  default:
    ;  
  }

  return logQueue != 0
         && ((currentProcess == 'c' && config.logCognition)
             || (currentProcess == 'm' && config.logMotion));
}

void LogCtrl::initInfoRequests()
{
  debugRequest("representation:GameInfo");
  debugRequest("representation:RobotInfo");
}

void LogCtrl::initQueue()
{
  if(logQueue != 0)
    return;
  logQueue = new LogQueue(config.logFileName.c_str(), config.queueSize);
  ASSERT(logQueue != 0);
  logQueue->unfilter(idProcessBegin);
  logQueue->unfilter(idProcessFinished);

  for(std::vector<std::string>::const_iterator j = config.logMessages.begin(); j != config.logMessages.end(); ++j)
  {
    std::string msg = *j;
    bool once = false;
    if(msg[0] == '1')
    {
      once = true;
      msg = msg.substr(1);
    }
    for(int i = 0; i < numOfMessageIDs; ++i)
    {
      if(msg == ::getName((MessageID)i))
      {
        if((MessageID)i == idXabslDebugSymbols)
          debugRequest("automated requests:xabsl:debugSymbols", once);
        else if((MessageID)i == idXabslDebugMessage)
          debugRequest("automated requests:xabsl:debugMessages", once);
        else
          debugRequest("representation:" + msg, once);
        logQueue->unfilter((MessageID)i);
      }
    }
  }
}

void LogCtrl::debugRequest(std::string name, bool once)
{
  DebugRequest d;
  d.description = name;
  d.enable = true;
  d.once = once;
  tmpQueue.out.bin << d;
  tmpQueue.out.finishMessage(idDebugRequest);

  tmpQueue.handleAllMessages(*debug);
  tmpQueue.clear();
}
