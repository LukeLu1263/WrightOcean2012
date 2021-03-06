/** 
* @file Simulation/Geometries/Geometry.h
* Declaration of class Geometry
* @author Colin Graf
*/

#pragma once

#include <ode/ode.h>

#ifdef WIN32
#include <unordered_map>
#else
#include <tr1/unordered_map>
#endif

#include "Simulation/PhysicalObject.h"

/**
* @class Geometry
* Abstract class for geometries of physical objects
*/
class Geometry : public PhysicalObject
{
public:
  float approxRadius; /**< The radius of a sphere that approximates the size of the geometry. This radius is used for implementing a fuzzy but fast distance sensor */
  float approxRadiusSqr; /**< precomputed square of \c approxRadius */

  /**
  * @class Material
  * Describes friction properties of the material a geometry is made of
  */
  class Material : public Element
  {
  public:
    std::string name; /**< The name of the material */
    std::tr1::unordered_map<std::string, float> frictions; /**< The friction of the material on another material */
    std::tr1::unordered_map<std::string, float> rollingFrictions; /**< The rolling friction of the material on another material */

    /**
    * Looks up the friction on another material
    * @param other The other material
    * @param friction The friction on the other material
    * @return Whether there is a friction specified or not
    */
    bool getFriction(const Material& other, float& friction) const;

    /**
    * Looks up the rolling friction on another material
    * @param other The other material
    * @param rollingFriction The rolling friction on the other material
    * @return Whether there is a rolling friction specified or not
    */
    bool getRollingFriction(const Material& other, float& rollingFriction) const;

  private:
    mutable std::tr1::unordered_map<const Material*, float> materialToFriction; /**< A pointer map to speed up friction lookups */
    mutable std::tr1::unordered_map<const Material*, float> materialToRollingFriction; /**< A pointer map to speed up rolling friction lookups */

    /** 
    * Registers an element as parent
    * @param element The element to register
    */
    virtual void addParent(Element& element);
  };

  float color[4]; /**< A color for drawing the geometry */
  Material* material; /**< The material the surface of the geometry is made of */

  /** Default constructor */
  Geometry() : material(0), created(false)
  {
    color[0] = color[1] = color[2] = 0.8f;
    color[3] = 1.0f;
  }

  /** 
  * Creates the geometry (not including \c translation and \c rotation)
  * @param space A space to create the geometry in
  * @return The created geometry
  */
  virtual dGeomID createGeometry(dSpaceID space);

private:
  bool created;

  /**
  * Draws physical primitives of the object (including children) on the currently selected OpenGL context
  * @param flags Flags to enable or disable certain features
  */
  virtual void drawPhysics(unsigned int flags) const;
};
