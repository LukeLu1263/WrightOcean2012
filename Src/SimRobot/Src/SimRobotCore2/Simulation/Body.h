/** 
* @file Simulation/Body.h
* Declaration of class Body
* @author Colin Graf
*/

#pragma once

#include <ode/ode.h>

#include "Simulation/PrimaryObject.h"

class Mass;
class Geometry;

/**
* @class Body
* A movable rigid body
*/
class Body : public PrimaryObject
{
public:
  dBodyID body;
  Body* parentBody; /**< The parent body in chain of bodies (might be 0) */
  Body* rootBody; /**< The first movable body in a chain of bodies (might point to itself) */
  dMass mass; /**< The mass of the body (at \c centerOfMass)*/

  /** Default constructor */
  Body() : body(0), bodySpace(0) {mass.mass = 0.f;}

  /**
  * Prepares the object and the currently selected OpenGL context for drawing the object. 
  * Loads textures and creates display lists. Hence, this function is called for each OpenGL 
  * context the object should be drawn in. */
  virtual void createGraphics();

  /**
  * Draws physical primitives of the object (including children) on the currently selected OpenGL context
  * @param flags Flags to enable or disable certain features
  */
  virtual void drawPhysics(unsigned int flags) const;

  /** Draws appearance primitives of the object (including children) on the currently selected OpenGL context (as fast as possible) */
  virtual void drawAppearances() const;

  /** Updates the transformation from the parent to this body (since the pose of the body may have changed) */
  void updateTransformation();

  /** Moves the object and its children relative to its current position
  * @param offset The distance to move
  */
  void move(const Vector3<>& offset);

  /**
  * Rotate the object and its children around a point
  * @param rotation The rotation offset
  * @param point The point to turn around
  */
  void rotate(const Matrix3x3<>& rotation, const Vector3<>& point);

  /**
  * Enables or disables the physics simulation of the body
  * @param enable Whether to enable or disable the physics simulation
  */
  void enablePhysics(bool enable);

  // API
  virtual void resetDynamics();

protected:
  std::list<dGeomID> geometries; /**< List of geometries attached to the body */
  
  /** Destructor */
  virtual ~Body();

  /** 
   * Creates the physical objects used by the OpenDynamicsEngine (ODE).
   * These are a geometry object for collision detection and/or a body,
   * if the simulation object is movable.
   */
  virtual void createPhysics();
  
private:
  Vector3<> centerOfMass; /**< The position of the center of mass relative to the pose of the body */
  float centerOfMassTransformation[16];

  dSpaceID bodySpace; /**< The collision space for a connected group of movable objects */

  std::list<Body*> bodyChildren; /**< List of first-degree child bodies that are connected to this body over a joint */

  /**
  * Creates a ODE geometry and attaches it to the body
  * @param parentOffset the base geometry offset from the center of mass of the body
  * @param geometry A geometry description
  */
  void addGeometry(const Pose3<>& parentOffset, Geometry& geometry);

  /**
  * Adds a mass to the mass of the body
  * @param mass A mass description of the mass to add
  */
  void addMass(Mass& mass);

  /** 
  * Registers an element as parent
  * @param element The element to register
  */
  virtual void addParent(Element& element);

  // API
  virtual const float* getPosition() const;
  virtual const float* getRotation() const;
  virtual void move(const float* pos, const float* rot);
  virtual void move(const float* pos);
};
