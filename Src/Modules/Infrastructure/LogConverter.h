/**
 * @file LogConverter.h
 *
 * @author reich
 * @author <a href="afabisch@tzi.de>Alexander Fabisch</a>
 */

#pragma once

#include "Tools/Streams/Streamable.h"
#include "Tools/MessageQueue/InMessage.h"
#include "Tools/MessageQueue/MessageQueue.h"

class LogConverter
{
private:
  MessageQueue messageQueue;

  /** Conversion is required for logfiles recorded with r5703 or earlier. */
  bool isConvertRequiredSensorData(InMessage& message);
  /** After 83e227af3a178ad46028e4717607ef4d44a25c2c FrameInfo contains the cycle time. */
  bool isConvertRequiredFrameInfo(InMessage& message);
  /** After 83e227af3a178ad46028e4717607ef4d44a25c2c JointData contains the cycle time. */
  bool isConvertRequiredJointData(InMessage& message);

  Streamable* newConvertedSensorData(InMessage& message);
  Streamable* newConvertedFrameInfo(InMessage& message);
  Streamable* newConvertedJointData(InMessage& message);
  Streamable* newConvertedFilteredJointData(InMessage& message);
  std::size_t sizeofRepresentation(Streamable& streamable);

  std::size_t sizeofSensorDataRev5703;
  std::size_t sizeofFrameInfo_83e22;
  std::size_t sizeofJointData_83e22;
public:
  LogConverter();
  virtual ~LogConverter();

  Streamable* newConvertedRepresentation(InMessage& message, const int representationId);
  bool isConvertRequired(InMessage& message, const int representationId);

};
