/** 
* @file Simulation/Sensor.cpp
* Implementation of class Actuator
* @author Colin Graf
*/

#include "Sensor.h"
#include "CoreModule.h"
#include "SensorWidget.h"

const QIcon* Sensor::getIcon() const
{
  return &CoreModule::module->sensorIcon;
}

SimRobot::Widget* Sensor::createWidget()
{
  return new SensorWidget(this);
}

SimRobotCore2::Sensor::Data Sensor::getValue()
{
  if(lastSimulationStep != Simulation::simulation->simulationStep)
  {
    updateValue();
    lastSimulationStep = Simulation::simulation->simulationStep;
  }
  return data;
}