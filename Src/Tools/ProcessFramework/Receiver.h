/**
 * @file Tools/ProcessFramework/Receiver.h
 *
 * This file declares classes related to receivers.
 *
 * @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas R�fer</a>
 */

#pragma once

#include "PlatformProcess.h"
#include "Tools/Streams/InStreams.h"

/**
 * The class is the base class for receivers.
 * A receiver is an object that reads packages from Aperios queues.
 * The class manages a global list of all receivers in an Aperios process.
 */
class ReceiverList
{
private:
  ReceiverList* next;         /**< The successor of the current receiver. */
  std::string name;           /**< The name of a receiver without the module's name. */

protected:
  PlatformProcess* process;   /**< The process this receiver is associated with. */
  int eventId;                /**< The id of the current receiver in the range [0..30]. */
  bool blocking;              /**< Stores whether this is a blocking receiver. */
  void* package[3];           /**< A triple buffer for received packages. */
  volatile int reading;       /**< Index of package reserved for reading. */
  volatile int actual;        /**< Index of package that is the most actual. */

  /**
   * The function checks whether a new package has arrived.
   */
  virtual void checkForPackage() = 0;

public:
  /**
   * The constructor.
   * @param process The process this receiver is associated with.
   * @param receiverName The Aperios connection name of the receiver without the process name.
   * @param blocking Decides whether this receiver blocks the execution of the next frame
   *                 until it has received a package.
   */
  ReceiverList(PlatformProcess* process, const std::string& receiverName, bool blocking);

  /**
   * Destructor.
   */
  virtual ~ReceiverList();

  /**
   * Returns the begin of the list of all receivers.
   * @return The first receiver in the list, or 0 if the list ist empty.
   */
  ReceiverList*& getFirst();

  /**
   * Returns the next receiver in the list.
   * @return The next receiver in the list, or 0 if this receiver is the last one.
   */
  ReceiverList* getNext() const {return next;}

  /**
   * Returns the Aperios connection name of the receiver.
   * @return The Aperios connection name without the process name ("Receiver.type.O")
   */
  const std::string& getName() const {return name;}

  /**
   * Returns whether a new package was received in the current frame.
   * This is always true if this is a blocking receiver.
   * @return Has a new package been received?
   */
  bool receivedNew() const;

  /**
   * The function must be called to finish the current frame.
   */
  void finishFrame();

  /**
   * The function checks for all receivers whether a new package has arrived.
   */
  void checkAllForPackages();

  /**
   * The function sets the package.
   * @param p The package.
   */
  void setPackage(void* p);

  /**
   * The function determines whether the receiver has a pending package.
   * @return Is there still an unprocessed package?
   */
  bool hasPendingPackage() const {return package[actual] != 0;}

  /**
   * The function searches for a receiver with the given name.
   * @param processName The name of the receiver's process.
   * @param receiverName The name of the requested receiver.
   * @return If the receiver is found, a pointer to it is returned.
   *         Otherwise, the funtion returns 0.
   */
  ReceiverList* lookup(const std::string& processName, const std::string& receiverName);
};

/**
 * The class implements a receiver.
 * A receiver is an object that reads packages from an Aperios queue.
 */
template<class T> class Receiver : public ReceiverList, public T
{
private:
  /**
   * The function checks whether a new package has arrived.
   */
  virtual void checkForPackage()
  {
    reading = actual;
    if(package[reading])
    {
      T& data = *static_cast<T*>(this);
      InBinaryMemory memory(package[reading]);
      memory >> data;
      delete [](char*) package[reading];
      package[reading] = 0;
      process->setEventId(eventId);
    }
  }

public:
  /**
   * The constructor.
   * @param process The process this receiver is associated with.
   * @param receiverName The Aperios connection name of the receiver without the process name.
   * @param blocking Decides whether this receiver blocks the execution of the next frame
   *                 until it has received a package.
   */
  Receiver(PlatformProcess* process, const std::string& receiverName, bool blocking)
    : ReceiverList(process, receiverName, blocking)
  {
  }
};
