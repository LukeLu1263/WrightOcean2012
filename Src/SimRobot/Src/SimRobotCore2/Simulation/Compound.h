/** 
* @file Simulation/Compound.h
* Declaration of class Compound
* @author Colin Graf
*/

#pragma once

#include "Simulation/PrimaryObject.h"

class Geometry;

/**
* @class Compound
* A non-movable physical object
*/
class Compound : public PrimaryObject
{
private:
  std::list<dGeomID> geometries; /**< List of geometries attached to the body */

  /** 
  * Creats the physical objects used by the OpenDynamicsEngine (ODE).
  * These are a geometry object for collision detection and/or a body,
  * if the simulation object is movable.
  */
  virtual void createPhysics();

  /**
  * Creates a stationary ODE geometry
  * @param parentPose The pose of the group or geometry
  * @param geometry A geometry description
  */
  void addGeometry(const Pose3<>& parentPose, Geometry& geometry);

  /**
  * Draws physical primitives of the object (including children) on the currently selected OpenGL context
  * @param flags Flags to enable or disable certain features
  */
  virtual void drawPhysics(unsigned int flags) const;

  /** Draws appearance primitives of the object (including children) on the currently selected OpenGL context (in order to create a display list) */
  virtual void assembleAppearances() const;
};
