/** 
* @file Simulation/Geometries/Geometry.cpp
* Implementation of class Geometry
* @author Colin Graf
*/

#include <GL/glew.h>

#include "Simulation/Geometries/Geometry.h"
#include "Tools/OpenGLTools.h"
#include "Platform/Assert.h"

dGeomID Geometry::createGeometry(dSpaceID space)
{
  if(!created)
  {
    OpenGLTools::convertTransformation(rotation, translation, transformation);
    created = true;
  }
  return 0;
}

void Geometry::drawPhysics(unsigned int flags) const
{
  glPushMatrix();
  glMultMatrixf(transformation);
  PhysicalObject::drawPhysics(flags);
  glPopMatrix();
}

void Geometry::Material::addParent(Element& element)
{
  Geometry* geometry = dynamic_cast<Geometry*>(&element);
  ASSERT(!geometry->material);
  geometry->material = this;
}

bool Geometry::Material::getFriction(const Material& other, float& friction) const
{
  {
    std::tr1::unordered_map<const Material*, float>::const_iterator iter = materialToFriction.find(&other);
    if(iter != materialToFriction.end())
    {
      friction = iter->second;
      return friction >= 0.f;
    }
  }

  friction = 0.f;
  int frictionValues = 0;

  {
    std::tr1::unordered_map<std::string, float>::const_iterator iter = frictions.find(other.name);
    if(iter != frictions.end())
    {
      friction += iter->second;
      ++frictionValues;
    }
  }

  {
    std::tr1::unordered_map<std::string, float>::const_iterator iter = other.frictions.find(name);
    if(iter != other.frictions.end())
    {
      friction += iter->second;
      ++frictionValues;
    }
  }

  bool frictionDefined = frictionValues > 0;
  if(frictionDefined)
    friction /= float(frictionValues);
  else
    friction = -1.f;

  materialToFriction[&other] = friction;
  return frictionDefined;
}

bool Geometry::Material::getRollingFriction(const Material& other, float& rollingFriction) const
{
  {
    std::tr1::unordered_map<const Material*, float>::const_iterator iter = materialToRollingFriction.find(&other);
    if(iter != materialToRollingFriction.end())
    {
      rollingFriction = iter->second;
      return rollingFriction >= 0.f;
    }
  }

  {
    std::tr1::unordered_map<std::string, float>::const_iterator iter = rollingFrictions.find(other.name);
    if(iter != rollingFrictions.end())
    {
      rollingFriction = iter->second;
      materialToRollingFriction[&other] = rollingFriction;
      return true;
    }
  }

  rollingFriction = -1.f;
  materialToRollingFriction[&other] = rollingFriction;
  return false;
}
