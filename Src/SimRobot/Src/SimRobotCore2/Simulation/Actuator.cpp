/** 
* @file Simulation/Actuator.cpp
* Implementation of class Actuator
* @author Colin Graf
*/

#include "Actuator.h"
#include "CoreModule.h"

const QIcon* Actuator::getIcon() const
{
  return &CoreModule::module->actuatorIcon;
}

SimRobot::Widget* Actuator::createWidget()
{
  CoreModule::module->application->openObject(CoreModule::module->actuatorsObject);
  if(ActuatorsWidget::actuatorsWidget)
    ActuatorsWidget::actuatorsWidget->openActuator(fullName);
  return 0;
}
