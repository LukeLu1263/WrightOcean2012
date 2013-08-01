/**
* @file Tools/ProcessFramework/PlatformProcess.h
*
* This file declares the base class for processes.
*
* @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
*/

#pragma once

#include "Platform/Thread.h"
#include "Platform/SystemCall.h"
#include "Platform/Semaphore.h"

class SenderList;
class ReceiverList;

/**
* This class is the base class for all processes.
*/
class PlatformProcess
{
private:
  DECLARE_SYNC;
  SenderList* firstSender; /**< The begin of the list of all senders of this process. */
  ReceiverList* firstReceiver; /**< The begin of the list of all receivers of this process. */
  int blockMask, /**< A mask with bits set for all blocking senders and receivers. */
      eventMask; /**< A mask with bits set for all senders and receivers that received an event */
  int lastTime; /**< The last time when Process::Main() was finished. */
  int id; /**< A counter that is used to provide different ids to senders and receivers. */
  unsigned sleepUntil; /**< The process should sleep until this point in time is reached. */
  int priority; /**< The priority of the process. */
  Semaphore sem; /**< The semaphore is triggered whenever this process receives new data. */

protected:
  /**
  * The function will be call from the process framework in each frame.
  * It shall provide the main functionality of the process.
  * @return If 0 is returned, the function main will not be called on a timed basis
  *         again. A positive number specifies the minimum number of microseconds to
  *         wait before the next frame starts. A negative number specifies a cycle
  *         time, i.e. the time between two activations of the processMain() function.
  */
  virtual int processMain() = 0;

public:

  /**
  * Constructor.
  */
  PlatformProcess() :
    firstSender(0),
    firstReceiver(0),
    blockMask(0),
    eventMask(0),
    lastTime(0),
    id(0),
    sleepUntil(0),
    priority(0)
  {
  }

  /**
  * Virtual destructor.
  */
  virtual ~PlatformProcess() {}

  /**
  * The function returns a new id each time it is called.
  * @return The next unused id.
  */
  int getNextId() {return id++;}

  /**
  * The function returns the begin of list of all senders.
  * Note that the function returns a reference that can be changed.
  * @return A reference to the address of the first element.
  */
  SenderList*& getFirstSender() {return firstSender;}

  /**
  * The function returns the begin of list of all receivers.
  * Note that the function returns a reference that can be changed.
  * @return A reference to the address of the first element.
  */
  ReceiverList*& getFirstReceiver() {return firstReceiver;}

  /**
  * The functions sets or resets a bit in the blocking mask.
  * After a bit is set in the blocking mask for a certain
  * sender or receiver, a new frame will not be started before
  * this sender or receiver received an event.
  * @param id The id of the sender or receiver.
  * @param block Should it block or not?
  */
  void setBlockingId(int id, bool block = true);

  /**
  * The function is called when an event was received.
  * If this was the last event the process was waiting for, the next
  * frame is started, i.e. NextFrame() is called.
  * @param id The id of the sender or receiver that received an event.
  */
  void setEventId(int id);

  /**
  * Returns the event mask.
  * @return The event mask.
  */
  int getEventMask() {return eventMask;}

  /**
  * Resets the event mask.
  */
  void resetEventMask() {eventMask = 0;}

  /**
  * The function is called once for each frame.
  */
  void nextFrame();

  /**
  * The function checks whether a timeout condition is satisfied if one was defined.
  * If this is the case, the corresponding eventId is set.
  */
  void checkTime();

  /**
  * The function returns whether the process is waiting for any event.
  * If it is not waiting for an event, it can terminate.
  * @return Waiting for any event (sender, receiver, or time)?
  */
  bool isWaiting() const
  {
    return blockMask != 0;
  }

  /**
  * The function sets the priority of the process.
  * @attention The priority can only be set in the constructor of the process.
  *            Any further changes will be ignored.
  * @param priority The new priority. Reasonable values are -2 .. 2 and 15.
  */
  void setPriority(int priority) {this->priority = priority;}

  /**
  * The function determines the priority of the process.
  * @return The priority of the process.
  */
  int getPriority() const {return priority;}

  /**
  * The method is called when the process is terminated.
  */
  virtual void terminate() {}

  /**
  * The method returns the amount of time the process should wait.
  * @return The period in ms. If -1, no delay was set.
  */
  int getDelay() const
  {
    if(sleepUntil)
    {
      int toSleep = int(sleepUntil - SystemCall::getCurrentSystemTime());
      return toSleep <= 0 ? 0 : toSleep;
    }
    return -1;
  }

  /**
  * The method waits forever or until package was received.
  */
  void wait() {sem.wait();}

  /**
  * The method waits for a certain amount of time or until package was received.
  * @param ms The number of ms to wait.
  */
  void wait(unsigned ms) {sem.wait(ms);}

  /**
  * The method has to be called to announce the receiption of a package.
  */
  void trigger() {sem.post();}
};
