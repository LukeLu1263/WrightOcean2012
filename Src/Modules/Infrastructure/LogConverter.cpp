/**
 * @file LogConverter.cpp
 *
 * @author reich
 * @author <a href="afabisch@tzi.de>Alexander Fabisch</a>
 */

#include <cstring>

#include "LogDataProvider.h"
#include "LogConverter.h"
#include "Platform/BHAssert.h"
#include "Representations/Infrastructure/SensorData.h"
#include "Representations/Infrastructure/Legacy/SensorDataRev5703.h"
#include "Representations/Infrastructure/JointData.h"
#include "Representations/Infrastructure/Legacy/JointData_83e22.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/Legacy/FrameInfo_83e22.h"
#include "Tools/MessageQueue/MessageIDs.h"
#include "Tools/Streams/OutStreams.h"

LogConverter::LogConverter()
{
  SensorDataRev5703 sensorDataRev5703;
  sizeofSensorDataRev5703 = sizeofRepresentation(sensorDataRev5703);
  JointData_83e22 jointData_83e22;
  sizeofJointData_83e22 = sizeofRepresentation(jointData_83e22);
  FrameInfo_83e22 frameInfo_83e22;
  sizeofFrameInfo_83e22 = sizeofRepresentation(frameInfo_83e22);
}

std::size_t LogConverter::sizeofRepresentation(Streamable& streamable)
{
  OutBinarySize outSize;
  outSize << streamable;
  return outSize.getSize();
}

LogConverter::~LogConverter() {}

/**
 * @param message the old representation destined for conversion
 * @param representationId The message id of the representation
 * @returns a newly allocated and converted representation
 */
Streamable* LogConverter::newConvertedRepresentation(InMessage& message, const int representationId)
{
  switch(representationId)
  {
  case idSensorData:
    return newConvertedSensorData(message);
  case idFrameInfo:
    return newConvertedFrameInfo(message);
  case idJointData:
    return newConvertedJointData(message);
  case idFilteredJointData:
    return newConvertedFilteredJointData(message);
  default:
    ASSERT(false);
  }
  return 0;
}

bool LogConverter::isConvertRequired(InMessage& message, const int representationId)
{
  switch(representationId)
  {
  case idSensorData:
    return isConvertRequiredSensorData(message);
  case idFrameInfo:
    return isConvertRequiredFrameInfo(message);
  case idJointData:
  case idFilteredJointData:
    return isConvertRequiredJointData(message);
  default:
    return false;
  }
}

bool LogConverter::isConvertRequiredSensorData(InMessage& message)
{
  return (std::size_t)message.getMessageSize() == sizeofSensorDataRev5703;
}

bool LogConverter::isConvertRequiredFrameInfo(InMessage& message)
{
  return (std::size_t)message.getMessageSize() == sizeofFrameInfo_83e22;
}

bool LogConverter::isConvertRequiredJointData(InMessage& message)
{
  return (std::size_t)message.getMessageSize() == sizeofJointData_83e22;
}

/*
 * @returns newly allocated and converted SensorData representation
 */
Streamable* LogConverter::newConvertedSensorData(InMessage& message)
{
  SensorDataRev5703 sensorDataRev5703;
  SensorData* sensorData = new SensorData;
  message.bin >> sensorDataRev5703;
  message.resetReadPosition();

  memcpy(sensorData->data, sensorDataRev5703.data, sizeof(float) * SensorDataRev5703::us);
  memcpy(&sensorData->data[SensorData::angleX], &sensorDataRev5703.data[SensorDataRev5703::angleX],
         sizeof(float) * (SensorData::numOfSensors - SensorData::angleX));
  memcpy(sensorData->currents, sensorDataRev5703.data, sizeof(sensorData->currents));
  memcpy(sensorData->temperatures, sensorDataRev5703.temperatures, sizeof(sensorData->temperatures));

  sensorData->timeStamp = sensorDataRev5703.timeStamp;
  sensorData->usActuatorMode = (SensorData::UsActuatorMode)sensorDataRev5703.usSensorType;
  sensorData->usTimeStamp = sensorData->timeStamp;

  return sensorData;
}

Streamable* LogConverter::newConvertedFrameInfo(InMessage& message)
{
  FrameInfo_83e22 frameInfo_83e22;
  FrameInfo* frameInfo = new FrameInfo;
  message.bin >> frameInfo_83e22;
  message.resetReadPosition();

  frameInfo->time = frameInfo_83e22.time;
  frameInfo->cycleTime = 0.01f;

  return frameInfo;
}

Streamable* LogConverter::newConvertedJointData(InMessage& message)
{
  JointData_83e22 jointData_83e22;
  JointData* jointData = new JointData;
  message.bin >> jointData_83e22;
  message.resetReadPosition();

  memcpy(jointData->angles, jointData_83e22.angles, sizeof(float) * JointData::numOfJoints);
  jointData->timeStamp = jointData_83e22.timeStamp;
  jointData->cycleTime = 0.01f; // hack

  return jointData;
}

Streamable* LogConverter::newConvertedFilteredJointData(InMessage& message)
{
  FilteredJointData_83e22 jointData_83e22;
  FilteredJointData* jointData = new FilteredJointData;
  message.bin >> jointData_83e22;
  message.resetReadPosition();

  memcpy(jointData->angles, jointData_83e22.angles, sizeof(float) * JointData::numOfJoints);
  jointData->timeStamp = jointData_83e22.timeStamp;
  jointData->cycleTime = 0.01f; // hack

  return jointData;
}
