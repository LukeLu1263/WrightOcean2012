/** 
* @file Simulation/Actuator.h
* Declaration of class Actuator
* @author Colin Graf
*/

#pragma once

#include <QString>

#include "SimRobotCore2.h"

/**
* @class Actuator
* An abstract class for actuators
*/
class Actuator : public SimRobotCore2::Actuator
{
public:
  QString fullName; /**< The path name to the object in the scene graph */

  /** Called before computing a simulation step to do something with the set-point of the actuator */
  virtual void act() = 0;

private:
  // API
  virtual const QString& getFullName() const {return fullName;}
  virtual const QIcon* getIcon() const;
  virtual SimRobot::Widget* createWidget();
};
