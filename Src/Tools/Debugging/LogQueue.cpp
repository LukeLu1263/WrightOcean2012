/**
* @file Tools/Debugging/LogQueue.cpp
*
* Provides a message Queue with filtering by message id and support for writing to a file
*
* @author <a href="mailto:matroc@tzi.de">Max Trocha</a>
*/

#include "LogQueue.h"
#include "Platform/BHAssert.h"

LogQueue::LogQueue(const char* fileName, int size)
{
  this->fileName = std::string(fileName);
  OutBinaryFile file(fileName);
  ASSERT(file.exists());
  writeAppendableHeader(file);
  fillThreshold = (int)(size * 0.9);
  setSize(size);
  init();
}

LogQueue::~LogQueue()
{
  save();
}

bool LogQueue::save()
{
  if(isEmpty())
    return false;

  OutBinaryFile file(fileName, true);
  ASSERT(file.exists());
  append(file);
  clear();
  return true;
}

bool LogQueue::handleMessage(InMessage& message)
{
  if(getStreamedSize() > fillThreshold)
    save();
  if(!filteredIDs[message.getMessageID()])
  {
    message >> *this;
    return true;
  }
  return false;
}

void LogQueue::init()
{
  clear();
  for(int i = 0; i < numOfMessageIDs; ++i)
  {
    filteredIDs[i] = true;
  }
}

void LogQueue::unfilter(MessageID messageID)
{
  filteredIDs[messageID] = false;
}

void LogQueue::filter(MessageID messageID)
{
  filteredIDs[messageID] = true;
}
