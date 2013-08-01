/** 
* @file Simulation/PhysicalObject.h
* Declaration of class PhysicalObject
* @author Colin Graf
*/

#pragma once

#include "Simulation/SimObject.h"
#include "Tools/Pose3.h"

/**
* @class PhysicalObject
* Abstract class for scene graph objects with physical representation
*/
class PhysicalObject : public SimObject
{
public:
  Pose3<> pose; /**< The absolute pose of the object */
  std::list<PhysicalObject*> children; /**< List of subordinate physical scene graph objects */
  std::list<PhysicalObject*> drawings; /**< List of subordinate physical objects that will be drawn relative to this one */

  /** 
  * Creats the physical objects used by the OpenDynamicsEngine (ODE).
  * These are a geometry object for collision detection and/or a body,
  * if the simulation object is movable.
  */
  virtual void createPhysics();

  /**
  * Draws physical primitives of the object (including children) on the currently selected OpenGL context
  * @param flags Flags to enable or disable certain features
  */
  virtual void drawPhysics(unsigned int flags) const;

protected:
  /** 
  * Registers an element as parent
  * @param element The element to register
  */
  virtual void addParent(Element& element);

private:
  std::list<SimRobotCore2::Controller3DDrawing*> controllerDrawings; /**< Drawings registered by another SimRobot module */

  // API
  virtual bool registerDrawing(SimRobotCore2::Controller3DDrawing& drawing);
  virtual bool unregisterDrawing(SimRobotCore2::Controller3DDrawing& drawing);
};
