/**
* @file  Platform/linux/SoundPlayer.cpp
* Implementation of class SoundPlayer.
* @attention this is the Linux implementation
* @author Colin Graf
*/

#include <csignal>
#include <sys/types.h>
#include <sys/wait.h>
#include <cstdlib>
#include <cstdio>
#include <QSound>

#include "SoundPlayer.h"
#include "Platform/File.h"

SoundPlayer SoundPlayer::soundPlayer;

SoundPlayer::SoundPlayer(): Thread<SoundPlayer>(),
  started(false), closing(false) {}

SoundPlayer::~SoundPlayer()
{
  if(started)
  {
    closing = true;
    sem.post();
  }
}

void SoundPlayer::start()
{
  Thread<SoundPlayer>::start(this, &SoundPlayer::main);
}

void SoundPlayer::main()
{
  while(isRunning() && !closing)
  {
    flush();
    VERIFY(sem.wait());
  }
}

void SoundPlayer::playDirect(const std::string& basename)
{
  std::string fileName(filePrefix);
  fileName += basename;
  QSound::play(fileName.c_str());
}


void SoundPlayer::flush()
{
  for(;;)
  {
    std::string first;
    {
      SYNC;
      if(0 == queue.size())
        break;
      first = queue.front();
      queue.pop_front();
    }

    playDirect(first);
  }
}


int SoundPlayer::play(const std::string& name)
{
  int queuelen;

  {
    SYNC_WITH(soundPlayer);
    soundPlayer.queue.push_back(name.c_str()); // avoid copy-on-write
    queuelen = soundPlayer.queue.size();
    if(!soundPlayer.started)
    {
      soundPlayer.started = true;
      soundPlayer.filePrefix = File::getBHDir();
      soundPlayer.filePrefix += "/Config/Sounds/";
      soundPlayer.start();
    }
    else
      soundPlayer.sem.post();
  }
  return queuelen;
}
