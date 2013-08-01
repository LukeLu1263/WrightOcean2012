/**
* @file Tools/ProcessFramework/PlatformProcess.cpp
*
* This file implements the base class for processes.
*
* @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
*/

#include "PlatformProcess.h"
#include "Receiver.h"
#include "Sender.h"

void PlatformProcess::setBlockingId(int id, bool block)
{
  if(block)
    blockMask |= 1 << id;
  else
    blockMask &= ~(1 << id);
}

void PlatformProcess::setEventId(int id)
{
  eventMask |= 1 << id;
  if(!id || (blockMask && (eventMask & blockMask)))
  {
    blockMask = 0;
    nextFrame();
  }
}

void PlatformProcess::nextFrame()
{
  int frameTime = processMain();
  if(getFirstSender())
    getFirstSender()->finishFrame();
  if(getFirstReceiver())
    getFirstReceiver()->finishFrame();
  resetEventMask();
  int currentTime = SystemCall::getCurrentSystemTime();
  if(frameTime < 0)
  {
    int toWait = lastTime - frameTime - currentTime;
    if(toWait < 0)
    {
      toWait = 0;
      lastTime = currentTime;
    }
    else
      lastTime -= frameTime;
    sleepUntil = currentTime + toWait;
  }
  else if(frameTime > 0)
    sleepUntil = currentTime + frameTime;
  else
    sleepUntil = 0;
  setBlockingId(31, frameTime != 0);
}

void PlatformProcess::checkTime()
{
  if(sleepUntil && sleepUntil <= SystemCall::getCurrentSystemTime())
  {
    sleepUntil = 0;
    setEventId(31);
  }
}
