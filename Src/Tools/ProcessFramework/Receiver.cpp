/**
 * @file Tools/ProcessFramework/Receiver.cpp
 *
 * This file implements classes related to receivers.
 *
 * @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#include "ProcessFramework.h"

ReceiverList::ReceiverList(PlatformProcess* p , const std::string& receiverName, bool blocking) :
  next(0),
  name(receiverName), // copy the receiver's name. The name of the process is still missing.
  process(p),
  eventId(process->getNextId()),
  blocking(blocking),
  reading(0),
  actual(0)
{
  if(getFirst())
  {
    ReceiverList* p = getFirst();
    while(p->next)
      p = p->next;
    p->next = this;
  }
  else
    getFirst() = this;
  for(int i = 0; i < 3; ++i)
    package[i] = 0;
}

ReceiverList::~ReceiverList()
{
  for(int i = 0; i < 3; ++i)
    if(package[i])
      delete [] (char*) package[i];
}

ReceiverList*& ReceiverList::getFirst()
{
  return process->getFirstReceiver();
}

bool ReceiverList::receivedNew() const
{
  return (process->getEventMask() & 1 << eventId) != 0;
}

void ReceiverList::finishFrame()
{
  for(ReceiverList* p = getFirst(); p; p = p->getNext())
    process->setBlockingId(p->eventId, p->blocking);
}

void ReceiverList::checkAllForPackages()
{
  for(ReceiverList* p = getFirst(); p; p = p->getNext())
    p->checkForPackage();
}

ReceiverList* ReceiverList::lookup(const std::string& processName, const std::string& receiverName)
{
  for(ReceiverList* p = getFirst(); p; p = p->getNext())
    if(processName + "." + p->name == receiverName)
      return p;
  return 0;
}

void ReceiverList::setPackage(void* p)
{
  int writing = 0;
  if(writing == actual)
    ++writing;
  if(writing == reading)
    if(++writing == actual)
      ++writing;
  ASSERT(writing != actual);
  ASSERT(writing != reading);
  if(package[writing])
    delete [] (char*) package[writing];
  package[writing] = p;
  actual = writing;
  process->trigger();
}
