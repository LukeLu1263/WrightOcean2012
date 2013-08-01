/** 
* @file Simulation/Masses/Mass.h
* Declaration of class Mass
* @author Colin Graf
*/

#pragma once

#include <ode/ode.h>

#include "Simulation/PhysicalObject.h"

/**
* @class Mass
* Abstract class for masses of physical objects
*/
class Mass : public PhysicalObject
{
public:
  /** Default constructor */
  Mass() : created(false) {}

  /** 
  * Creates the mass of a physical object (including children and not including \c translation and \c rotation)
  * @return The mass
  */
  const dMass& createMass();

protected:
  dMass mass;
  bool created;

  /** Creates the mass (not including children, \c translation or \c rotation) */
  virtual void assembleMass();
};
