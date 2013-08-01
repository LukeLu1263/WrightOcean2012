/** 
* @file Simulation/Sensors/Bumper.h
* Declaration of class Bumper
* @author Thomas Röfer
*/

#pragma once

#include "Simulation/Body.h"
#include "Simulation/Sensor.h"

/**
* @class CollisionSensor
* The collision sensor interface.
* Note that this interface is used from class Simulation.
*/
class CollisionSensor : public Sensor
{
public:
  /** The class Simulation sets whether there is a collision or not. */
  void set(bool collision) {this->collision = collision;}
  
private:
  bool collision;
  
  //API
  virtual void updateValue() {data.boolValue = collision;}
  virtual bool getMinAndMax(float& min, float& max) const {return false;}
};

/**
* @class Bumper
* A sensor that detects contact to other objects. Technically, the sensor
* is a body and it detects whether its sub-objects have collisions with other
* objects.
*/
class Bumper : public Body
{
public:
  /** Default constructor */
  Bumper();

private:
  CollisionSensor sensor;
  
  /** 
  * Creats the physical objects used by the OpenDynamicsEngine (ODE).
  * These are a geometry object for collision detection and/or a body,
  * if the simulation object is movable.
  */
  virtual void createPhysics();

  /** Registers this object with children, actuators and sensors at SimRobot's GUI. */
  virtual void registerObjects();
};
