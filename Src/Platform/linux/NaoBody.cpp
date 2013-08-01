/**
* @file Platform/linux/NaoBody.cpp
* Declaration of a class for accessing the body of the nao via NaoQi/libbhuman.
* @author Colin Graf
* @author jeff
*/

#include <sys/mman.h>
#include <fcntl.h>
#include <semaphore.h>
#include <unistd.h>
#include <pthread.h>
#include <cerrno>
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <cstdarg>

#include "NaoBody.h"
#include "BHAssert.h"

#include "libbhuman/bhuman.h"

class NaoBodyAccess
{
public:
  int fd; /**< The file descriptor for the shared memory block. */
  sem_t* sem; /**< The semaphore used for synchronizing to the NaoQi DCM. */
  LBHData* lbhData; /**< The pointer to the mapped shared memory block. */

  NaoBodyAccess() : fd(-1), sem(SEM_FAILED), lbhData((LBHData*)MAP_FAILED) {}

  ~NaoBodyAccess()
  {
    cleanup();
  }

  bool init()
  {
    if(lbhData != MAP_FAILED)
      return true;

    fd = shm_open(LBH_MEM_NAME, O_RDWR, S_IRUSR | S_IWUSR);
    if(fd == -1)
      return false;

    sem = sem_open(LBH_SEM_NAME, O_RDWR, S_IRUSR | S_IWUSR, 0);
    if(sem == SEM_FAILED)
    {
      close(fd);
      fd = -1;
      return false;
    }

    VERIFY((lbhData = (LBHData*)mmap(NULL, sizeof(LBHData), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0)) != MAP_FAILED);
    lbhData->state = okState;
    return true;
  }

  void cleanup()
  {
    if(lbhData != MAP_FAILED)
    {
      munmap(lbhData, sizeof(LBHData));
      lbhData = (LBHData*)MAP_FAILED;
    }
    if(fd != -1)
    {
      close(fd);
      fd = -1;
    }
    if(sem != SEM_FAILED)
    {
      sem_close(sem);
      sem = SEM_FAILED;
    }
  }

} naoBodyAccess;

NaoBody::NaoBody() :  writingActuators(-1), fdCpuTemp(-1), fdMbTemp(-1) {}

NaoBody::~NaoBody()
{
  if(fdCpuTemp != -1)
    close(fdCpuTemp);
  if(fdMbTemp != -1)
    close(fdMbTemp);
}

bool NaoBody::init()
{
  return naoBodyAccess.init();
}

void NaoBody::cleanup()
{
  naoBodyAccess.cleanup();
}

void NaoBody::setCrashed(int termSignal)
{
  ASSERT(naoBodyAccess.lbhData != (LBHData*)MAP_FAILED);
  naoBodyAccess.lbhData->state = BHState(termSignal);
}

bool NaoBody::wait()
{
  ASSERT(naoBodyAccess.lbhData != (LBHData*)MAP_FAILED);
  ASSERT(naoBodyAccess.sem != SEM_FAILED);
  do
  {
    if(sem_wait(naoBodyAccess.sem) == -1)
    {
      bool success = false;
      while(errno == 516)
      {
        if(sem_wait(naoBodyAccess.sem) == -1)
        {
          ASSERT(false);
          continue;
        }
        else
        {
          success = true;
          break;
        }
      }
      if(!success)
      {
        ASSERT(false);
        return false;
      }
    }
  }
  while(naoBodyAccess.lbhData->readingSensors == naoBodyAccess.lbhData->newestSensors);
  naoBodyAccess.lbhData->readingSensors = naoBodyAccess.lbhData->newestSensors;

  static bool shout = true;
  if(shout)
  {
    shout = false;
    printf("NaoQi is working\n");
  }

  return true;
}

const char* NaoBody::getName() const
{
  ASSERT(naoBodyAccess.lbhData != (LBHData*)MAP_FAILED);
  return naoBodyAccess.lbhData->robotName;
}

float* NaoBody::getSensors()
{
  ASSERT(naoBodyAccess.lbhData != (LBHData*)MAP_FAILED);
  return naoBodyAccess.lbhData->sensors[naoBodyAccess.lbhData->readingSensors];
}

void NaoBody::getTemperature(float& cpu, float& mb)
{
  cpu = mb = 0;

#ifndef NDEBUG
  if(fdCpuTemp == -1)
  {
    fdCpuTemp = open("/sys/class/i2c-adapter/i2c-1/1-004c/temp2_input", O_RDONLY);
    ASSERT(fdCpuTemp > -1);
  }

  if(fdMbTemp == -1)
  {
    fdMbTemp = open("/sys/class/i2c-adapter/i2c-1/1-004c/temp1_input", O_RDONLY);
    ASSERT(fdMbTemp > -1);
  }

  char buf[32];

  if(fdCpuTemp != -1)
  {
    VERIFY(lseek(fdCpuTemp, 0, SEEK_SET) == 0);
    int i = read(fdCpuTemp, buf, sizeof(buf));
    ASSERT(i != -1);
    if(i >= 0)
    {
      buf[i < int(sizeof(buf) - 1) ? i : int(sizeof(buf) - 1)] = '\0';
      cpu = float(atoi(buf)) / 1000.f;
    }
  }

  if(fdMbTemp != -1)
  {
    VERIFY(lseek(fdMbTemp, 0, SEEK_SET) == 0);
    int i = read(fdMbTemp, buf, sizeof(buf));
    ASSERT(i != -1);
    if(i >= 0)
    {
      buf[i < int(sizeof(buf) - 1) ? i : int(sizeof(buf) - 1)] = '\0';
      mb = float(atoi(buf)) / 1000.f;
    }
  }
#endif
}

bool NaoBody::getWlanStatus()
{
  return access("/sys/class/net/wlan0", F_OK) == 0;
}

void NaoBody::openActuators(float*& actuators)
{
  ASSERT(naoBodyAccess.lbhData != (LBHData*)MAP_FAILED);
  ASSERT(writingActuators == -1);
  writingActuators = 0;
  if(writingActuators == naoBodyAccess.lbhData->newestActuators)
    ++writingActuators;
  if(writingActuators == naoBodyAccess.lbhData->readingActuators)
    if(++writingActuators == naoBodyAccess.lbhData->newestActuators)
      ++writingActuators;
  ASSERT(writingActuators != naoBodyAccess.lbhData->newestActuators);
  ASSERT(writingActuators != naoBodyAccess.lbhData->readingActuators);
  actuators = naoBodyAccess.lbhData->actuators[writingActuators];
}

void NaoBody::closeActuators()
{
  ASSERT(naoBodyAccess.lbhData != (LBHData*)MAP_FAILED);
  ASSERT(writingActuators >= 0);
  naoBodyAccess.lbhData->newestActuators = writingActuators;
  writingActuators = -1;
}

