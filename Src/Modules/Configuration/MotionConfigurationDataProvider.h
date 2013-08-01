/**
* @file MotionConfigurationDataProvider.h
* This file declares a module that provides data loaded from configuration files.
* @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
*/

#pragma once

#include "Tools/Module/Module.h"
#include "Tools/MessageQueue/InMessage.h"
#include "Representations/Configuration/JointCalibration.h"
#include "Representations/Configuration/SensorCalibration.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Configuration/MassCalibration.h"
#include "Representations/Infrastructure/JointData.h"

MODULE(MotionConfigurationDataProvider)
  PROVIDES_WITH_MODIFY_AND_OUTPUT(JointCalibration)
  PROVIDES_WITH_MODIFY(SensorCalibration)
  PROVIDES_WITH_MODIFY_AND_OUTPUT(RobotDimensions)
  PROVIDES_WITH_MODIFY(MassCalibration)
  PROVIDES_WITH_MODIFY(HardnessSettings)
END_MODULE

class MotionConfigurationDataProvider : public MotionConfigurationDataProviderBase
{
private:
  JointCalibration* theJointCalibration;
  SensorCalibration* theSensorCalibration;
  RobotDimensions* theRobotDimensions;
  MassCalibration* theMassCalibration;
  HardnessSettings* theHardnessSettings;

  void update(JointCalibration& jointCalibration);
  void update(SensorCalibration& sensorCalibration);
  void update(RobotDimensions& robotDimensions);
  void update(MassCalibration& massCalibration);
  void update(HardnessSettings& hardnessSettings);

  void readJointCalibration();
  void readSensorCalibration();
  void readRobotDimensions();
  void readMassCalibration();
  void readHardnessSettings();

public:
  /**
  * Default constructor.
  */
  MotionConfigurationDataProvider();

  /**
  * Destructor.
  */
  ~MotionConfigurationDataProvider();
};
