/**
* @file Tools/Debugging/LogQueue.h
*
* Provides a message Queue with filtering by message id and support for writing to a file
*
* @author <a href="mailto:matroc@tzi.de">Max Trocha</a>
*/

#pragma once

#include "Tools/MessageQueue/MessageQueue.h"

class LogQueue : public MessageQueue, public MessageHandler
{
public:
  LogQueue(const char* fileName, int size);
  ~LogQueue();
  void init();
  void unfilter(MessageID messageID);
  void filter(MessageID messageID);
  bool save();
  bool handleMessage(InMessage& message);
private:
  bool filteredIDs[numOfMessageIDs];
  int fillThreshold;
  std::string fileName;
};
