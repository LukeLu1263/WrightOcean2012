/** 
* @file Simulation/Sensors/SingleDistanceSensor.h
* Declaration of class SingleDistanceSensor
* @author Colin Graf
*/

#pragma once

#include <ode/ode.h>

#include "Simulation/PhysicalObject.h"
#include "Simulation/Sensor.h"

/**
* @class SingleDistanceSensor
* A distance sensor that uses a ray to detect distances to other objects
*/
class SingleDistanceSensor : public PhysicalObject
{
public:
  float min; /**< The minimum distance the distance sensor can measure */
  float max; /**< The maximum distance the distance sensor can measure */

  /** Default constructor */
  SingleDistanceSensor();

private:
  /**
  * @class DistanceSensor
  * The distance sensor interface
  */
  class DistanceSensor : public Sensor
  {
  public:
    PhysicalObject* physicalObject; /** The physical object were the distance sensor is mounted on */
    dGeomID geom;
    float min;
    float max;
    float maxSqrDist;
    Pose3<> offset;

  private:
    float closestSqrDistance;
    dGeomID closestGeom;
    Pose3<> pose; /**< The pose of the sensor relative to the origin of the scene */

    static void staticCollisionCallback(DistanceSensor* sensor, dGeomID geom1, dGeomID geom2);
    static void staticCollisionWithSpaceCallback(DistanceSensor* sensor, dGeomID geom1, dGeomID geom2);

    //API
    virtual void updateValue();
    virtual bool getMinAndMax(float& min, float& max) const;
  } sensor;

  /** 
  * Creats the physical objects used by the OpenDynamicsEngine (ODE).
  * These are a geometry object for collision detection and/or a body,
  * if the simulation object is movable.
  */
  virtual void createPhysics();

  /** Registers this object with children, actuators and sensors at SimRobot's GUI. */
  virtual void registerObjects();

  /** 
  * Registers an element as parent
  * @param element The element to register
  */
  virtual void addParent(Element& element);

  /**
  * Draws physical primitives of the object (including children) on the currently selected OpenGL context
  * @param flags Flags to enable or disable certain features
  */
  virtual void drawPhysics(unsigned int flags) const;
};
