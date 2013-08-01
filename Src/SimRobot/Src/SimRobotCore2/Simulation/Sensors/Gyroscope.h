/** 
* @file Simulation/Sensors/Gyroscope.h
* Declaration of class Gyroscope
* @author Colin Graf
*/

#pragma once

#include "Simulation/SimObject.h"
#include "Simulation/Sensor.h"

class Body;

/**
* @class Gyroscope
* A simulated gyroscope sensor
*/
class Gyroscope : public SimObject
{
public:
  /** Default constructor */
  Gyroscope();

private:
  /**
  * @class GyroscopeSensor
  * The gyroscope sensor interface
  */
  class GyroscopeSensor : public Sensor
  {
  public:
    Body* body; /** The body were the gyroscope is mounted on */
    float angularVel[4]; /** The sensor reading */

    //API
    virtual void updateValue();
    virtual bool getMinAndMax(float& min, float& max) const {return false;}
  } sensor;
  
  /** 
  * Registers an element as parent
  * @param element The element to register
  */
  virtual void addParent(Element& element);

  /** Registers this object with children, actuators and sensors at SimRobot's GUI. */
  virtual void registerObjects();
};
