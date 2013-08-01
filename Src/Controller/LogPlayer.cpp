/**
* @file Controller/LogPlayer.cpp
*
* Implementation of class LogPlayer
*
* @author Martin Lötzsch
*/

#include "LogPlayer.h"
#include "Representations/Perception/JPEGImage.h"
#include "Representations/Infrastructure/Legacy/FrameInfo_83e22.h"
#include "Representations/Infrastructure/Legacy/JointData_83e22.h"
#include "Platform/SystemCall.h"
#include "Platform/BHAssert.h"

//If we don't have information on time per frame, assume we're running at 30hz
const int FALLBACK_TIME_PER_FRAME = 1000 / 30;

LogPlayer::LogPlayer(MessageQueue& targetQueue) :
  targetQueue(targetQueue)
{
  init();
}

void LogPlayer::init()
{
  clear();
  stop();
  numberOfFrames = 0;
  numberOfMessagesWithinCompleteFrames = 0;
  replayOffset = 0;
  lastRealTimestamp = SystemCall::getCurrentSystemTime();
  state = initial;
  loop = true; //default: loop enabled
}

bool LogPlayer::open(const char* fileName)
{
  InBinaryFile file(fileName);
  if(file.exists())
  {
    clear();
    file >> *this;
    stop();
    countFrames();
    return true;
  }
  return false;
}

void LogPlayer::play()
{
  if(state != playing)
    lastRealTimestamp = SystemCall::getCurrentSystemTime();
  state = playing;
}

void LogPlayer::stop()
{
  if(state == recording)
  {
    recordStop();
    return;
  }
  currentMessageNumber = -1;
  currentFrameNumber = -1;
  state = initial;
  lastImageFrameNumber = -1;
  currentLogTimestamp = 0;
  lastRealTimestamp = 0;
}

void LogPlayer::pause()
{
  if(getNumberOfMessages() == 0)
    state = initial;
  else
    state = paused;
}

void LogPlayer::stepBackward()
{
  pause();
  if(state == paused && currentFrameNumber > 0)
  {
    do
      queue.setSelectedMessageForReading(--currentMessageNumber);
    while(currentMessageNumber > 0 && queue.getMessageID() != idProcessFinished);
    --currentFrameNumber;
    stepRepeat();
  }
}

void LogPlayer::stepImageBackward()
{
  pause();
  if(state == paused && currentFrameNumber > 0)
  {
    int lastImageFrameNumber = this->lastImageFrameNumber;
    do
      stepBackward();
    while(lastImageFrameNumber == this->lastImageFrameNumber && currentFrameNumber > 0);
  }
}

void LogPlayer::stepForward()
{
  pause();
  if(state == paused && currentFrameNumber < numberOfFrames - 1 && currentMessageNumber < numberOfMessagesWithinCompleteFrames - 1)
  {
    do
    {
      copyMessage(++currentMessageNumber, targetQueue);
      unsigned tmpTime = getTimeInformation();
      if(tmpTime != 0)
        currentLogTimestamp = tmpTime;
      if(queue.getMessageID() == idImage || queue.getMessageID() == idJPEGImage)
        lastImageFrameNumber = currentFrameNumber;
    }
    while(queue.getMessageID() != idProcessFinished);
    ++currentFrameNumber;
  }
}

void LogPlayer::stepImageForward()
{
  pause();
  if(state == paused && currentFrameNumber < numberOfFrames - 1)
  {
    int lastImageFrameNumber = this->lastImageFrameNumber;
    do
      stepForward();
    while(lastImageFrameNumber == this->lastImageFrameNumber && currentFrameNumber < numberOfFrames - 1);
  }
}

void LogPlayer::stepRepeat()
{
  pause();
  if(state == paused && currentFrameNumber >= 0)
  {
    do
      queue.setSelectedMessageForReading(--currentMessageNumber);
    while(currentMessageNumber > 0 && queue.getMessageID() != idProcessFinished);
    --currentFrameNumber;
    stepForward();
  }
}

void LogPlayer::gotoFrame(int frame)
{
  pause();
  if(state == paused && frame < numberOfFrames)
  {
    currentFrameNumber = -1;
    currentMessageNumber = -1;
    currentLogTimestamp = logBeginTimestamp;
    while(++currentMessageNumber < getNumberOfMessages() && frame > currentFrameNumber + 1)
    {
      queue.setSelectedMessageForReading(currentMessageNumber);
      unsigned tmpTime = getTimeInformation();
      if(tmpTime != 0)
        currentLogTimestamp = tmpTime;
      if(queue.getMessageID() == idProcessFinished)
        ++currentFrameNumber;
    }
    stepForward();
  }
}

bool LogPlayer::save(const char* fileName)
{
  if(state == recording)
    recordStop();

  if(!getNumberOfMessages())
    return false;

  OutBinaryFile file(fileName);
  if(file.exists())
  {
    file << *this;
    return true;
  }
  return false;
}

bool LogPlayer::saveImages(const bool raw, const char* fileName)
{
  struct BmpHeader
  {
    unsigned k0, k1, k2;
    unsigned ofs1, ofs2;
    unsigned xsiz, ysiz;
    unsigned modbit;
    unsigned z1;
    unsigned len;
    unsigned z2, z3, z4, z5;
  } bmpHeader;

  char name[512];
  char fname[512];
  strcpy(name, fileName);
  if((strlen(name) > 4) && ((strncmp(name + strlen(name) - 4, ".bmp", 4) == 0) || (strncmp(name + strlen(name) - 4, ".jpg", 4) == 0)))
  {
    *(name + strlen(name) - 4) = 0;
  }
  if((strlen(name) > 4) && ((strncmp(name + strlen(name) - 4, "_000", 4) == 0) || (strncmp(name + strlen(name) - 4, "_001", 4) == 0)))
  {
    *(name + strlen(name) - 4) = 0;
  }
  int i = 0;
  for(currentMessageNumber = 0; currentMessageNumber < getNumberOfMessages(); currentMessageNumber++)
  {
    queue.setSelectedMessageForReading(currentMessageNumber);
    Image image;
    if(queue.getMessageID() == idImage)
    {
      in.bin >> image;
    }
    else if(queue.getMessageID() == idJPEGImage)
    {
      JPEGImage jpegImage;
      in.bin >> jpegImage;
      jpegImage.toImage(image);
    }
    else
      continue;

    Image rgbImage;
    if(!raw)
      rgbImage.convertFromYCbCrToRGB(image);

    sprintf(fname, "%s_%03i.bmp", name, i++);
    OutBinaryFile file(fname);
    if(file.exists())
    {
      int truelinelength = (3 * rgbImage.cameraInfo.resolutionWidth + 3) & 0xfffffc;
      char line[3 * 320 + 4];
      memset(line, 0, truelinelength);
      bmpHeader.k0 = 0x4d420000;  //2 dummy bytes 4 alignment, then "BM"
      bmpHeader.k1 = rgbImage.cameraInfo.resolutionHeight * truelinelength + 0x36;
      bmpHeader.k2 = 0;
      bmpHeader.ofs1 = 0x36;
      bmpHeader.ofs2 = 0x28;
      bmpHeader.xsiz = rgbImage.cameraInfo.resolutionWidth;
      bmpHeader.ysiz = rgbImage.cameraInfo.resolutionHeight;
      bmpHeader.modbit = 0x00180001;
      bmpHeader.z1 = 0;
      bmpHeader.len = truelinelength * rgbImage.cameraInfo.resolutionHeight;
      bmpHeader.z2 = 0;
      bmpHeader.z3 = 0;
      bmpHeader.z4 = 0;
      bmpHeader.z5 = 0;
      file.write(2 + (char*)&bmpHeader, sizeof(bmpHeader) - 2);
      for(int i = rgbImage.cameraInfo.resolutionHeight - 1; i >= 0; i--)
      {
        int ofs = 0;
        if(raw)
        {
          for(int j = 0; j < image.cameraInfo.resolutionWidth; j++)
          {
            line[ofs++] = image.image[i][j].cr;
            line[ofs++] = image.image[i][j].cb;
            line[ofs++] = image.image[i][j].y;
          }
        }
        else
        {
          for(int j = 0; j < rgbImage.cameraInfo.resolutionWidth; j++)
          {
            line[ofs++] = rgbImage.image[i][j].b;
            line[ofs++] = rgbImage.image[i][j].g;
            line[ofs++] = rgbImage.image[i][j].r;
          }
        }
        file.write(line, truelinelength);
      }
    }
    else
    {
      stop();
      return false;
    }
  }
  stop();
  return true;
}

void LogPlayer::recordStart()
{
  state = recording;
}

void LogPlayer::recordStop()
{
  while(getNumberOfMessages() > numberOfMessagesWithinCompleteFrames)
    removeLastMessage();
  currentMessageNumber = -1;
  currentFrameNumber = -1;
  state = initial;
}


void LogPlayer::setLoop(bool loop_)
{
  loop = loop_;
}

void LogPlayer::handleMessage(InMessage& message)
{
  if(state == recording)
  {
    message >> *this;
    if(message.getMessageID() == idProcessFinished)
    {
      numberOfMessagesWithinCompleteFrames = getNumberOfMessages();
      ++numberOfFrames;
    }
  }
}

bool LogPlayer::replay(bool realtime)
{
  doTick();
  if(state == playing)
  {
    if(currentFrameNumber < numberOfFrames - 1)
    {
      bool firstRun = true;
      unsigned timeToReplay = (SystemCall::getCurrentSystemTime() - lastRealTimestamp) - replayOffset;
      unsigned targetTimestamp = currentLogTimestamp + timeToReplay;
      while((!realtime && firstRun)
            || (realtime && currentLogTimestamp < targetTimestamp && currentFrameNumber < numberOfFrames - 1 && currentMessageNumber < numberOfMessagesWithinCompleteFrames - 1))
      {
        firstRun = false;
        do
        {
          copyMessage(++currentMessageNumber, targetQueue);
          unsigned tmpTime = getTimeInformation();
          if(tmpTime != 0)
            currentLogTimestamp = tmpTime;
          if(queue.getMessageID() == idImage || queue.getMessageID() == idJPEGImage)
            lastImageFrameNumber = currentFrameNumber;
        }
        while(queue.getMessageID() != idProcessFinished && currentMessageNumber < numberOfMessagesWithinCompleteFrames - 1);
        ++currentFrameNumber;
      }
      replayOffset = currentLogTimestamp - targetTimestamp;
      lastRealTimestamp = SystemCall::getCurrentSystemTime();
      if(currentFrameNumber == numberOfFrames - 1)
      {
        if(loop)  //restart in loop mode
        {
          gotoFrame(0);
          play();
        }
        else
          stop();
      }
      return true;
    }
    else
    {
      if(loop)  //restart in loop mode
      {
        gotoFrame(0);
        play();
      }
      else
        stop();
    }
  }
  return false;
}

void LogPlayer::keep(MessageID* messageIDs)
{
  LogPlayer temp((MessageQueue&) *this);
  moveAllMessages(temp);
  for(temp.currentMessageNumber = 0; temp.currentMessageNumber < temp.getNumberOfMessages(); ++temp.currentMessageNumber)
  {
    temp.queue.setSelectedMessageForReading(temp.currentMessageNumber);
    MessageID* m = messageIDs;
    while(*m)
    {
      if(temp.queue.getMessageID() == *m ||
         temp.queue.getMessageID() == idProcessBegin ||
         temp.queue.getMessageID() == idProcessFinished)
      {
        temp.copyMessage(temp.currentMessageNumber, *this);
        break;
      }
      ++m;
    }
  }
}

void LogPlayer::remove(MessageID* messageIDs)
{
  LogPlayer temp((MessageQueue&) *this);
  moveAllMessages(temp);
  for(temp.currentMessageNumber = 0; temp.currentMessageNumber < temp.getNumberOfMessages(); ++temp.currentMessageNumber)
  {
    temp.queue.setSelectedMessageForReading(temp.currentMessageNumber);
    MessageID* m = messageIDs;
    while(*m)
    {
      if(temp.queue.getMessageID() == *m)
        break;
      ++m;
    }
    if(!*m)
      temp.copyMessage(temp.currentMessageNumber, *this);
  }
}

void LogPlayer::statistics(int frequency[numOfDataMessageIDs])
{
  for(int i = 0; i < numOfDataMessageIDs; ++i)
    frequency[i] = 0;

  if(getNumberOfMessages() > 0)
  {
    int current = queue.getSelectedMessageForReading();
    for(int i = 0; i < getNumberOfMessages(); ++i)
    {
      queue.setSelectedMessageForReading(i);
      ASSERT(queue.getMessageID() < numOfDataMessageIDs);
      ++frequency[queue.getMessageID()];
    }
    queue.setSelectedMessageForReading(current);
  }
}

void LogPlayer::countFrames()
{
  fallbackTimings = false;
  numberOfFrames = 0;
  unsigned lastTime = 0;
  logBeginTimestamp = 0;
  for(int i = 0; i < getNumberOfMessages(); ++i)
  {
    queue.setSelectedMessageForReading(i);
    unsigned tmpTime = getTimeInformation();
    lastTime = tmpTime != 0 ? tmpTime : lastTime;
    if(logBeginTimestamp == 0 && lastTime != 0)
      logBeginTimestamp = lastTime;
    if(queue.getMessageID() == idProcessFinished)
    {
      ++numberOfFrames;
      numberOfMessagesWithinCompleteFrames = i + 1;
    }
  }
  if(lastTime == 0 || lastTime == logBeginTimestamp)
  {
    fallbackTimings = true;
    logBeginTimestamp = 0;
    logEndTimestamp = FALLBACK_TIME_PER_FRAME * numberOfFrames;
  }
  else
    logEndTimestamp = lastTime;
}

unsigned LogPlayer::getTimeInformation()
{
  if(fallbackTimings)
  {
    return currentFrameNumber == -1 ? 0 : FALLBACK_TIME_PER_FRAME * currentFrameNumber;
  }
  if(queue.getMessageID() == idFrameInfo)
  {
    FrameInfo_83e22 fi; // hack, usually we should read a FrameInfo object
    in.bin >> fi;
    queue.resetReadPosition();
    return fi.time;
  }
  else if(queue.getMessageID() == idImage)
  {
    Image img;
    in.bin >> img;
    queue.resetReadPosition();
    return img.timeStamp;
  }
  else if(queue.getMessageID() == idJPEGImage)
  {
    JPEGImage img;
    in.bin >> img;
    queue.resetReadPosition();
    return img.timeStamp;
  }
  else if(queue.getMessageID() == idJointData)
  {
    JointData_83e22 jd; // hack, usually we should read a JointData object
    in.bin >> jd;
    queue.resetReadPosition();
    return jd.timeStamp;
  }
  return 0;
}

void LogPlayer::setTime(int time)
{
  ASSERT(time >= 0);
  LogPlayerState stateBck = state;
  if(fallbackTimings)
    gotoFrame(time / FALLBACK_TIME_PER_FRAME);
  else
  {
    unsigned targetTime = (unsigned)time + logBeginTimestamp;
    if(targetTime < currentLogTimestamp)
    {
      currentFrameNumber = -1;
      currentMessageNumber = -1;
      currentLogTimestamp = logBeginTimestamp;
    }
    while(++currentMessageNumber < getNumberOfMessages() && targetTime > currentLogTimestamp)
    {
      queue.setSelectedMessageForReading(currentMessageNumber);
      unsigned tmpTime = getTimeInformation();
      if(tmpTime != 0)
        currentLogTimestamp = tmpTime;
      if(queue.getMessageID() == idProcessFinished)
        ++currentFrameNumber;
    }
    stepForward();
  }
  state = stateBck;
}

int LogPlayer::getTime()
{
  return (currentLogTimestamp - logBeginTimestamp);
}

int LogPlayer::getLength()
{
  return (logEndTimestamp - logBeginTimestamp);
}

void LogPlayer::setPause(bool pause)
{
  if(pause)
    this->pause();
  else
    this->play();
}
