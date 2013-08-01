/** 
* @file Simulation/Compound.cpp
* Implementation of class Compound
* @author Colin Graf
*/

#include <GL/glew.h>

#include "Simulation/Simulation.h"
#include "Simulation/Compound.h"
#include "Geometries/Geometry.h"
#include "Tools/ODETools.h"
#include "Tools/OpenGLTools.h"
#include "Platform/Assert.h"

void Compound::createPhysics()
{
  // create geometry
  for(std::list<PhysicalObject*>::const_iterator iter = children.begin(), end = children.end(); iter != end; ++iter)
  {
    Geometry* geometry = dynamic_cast<Geometry*>(*iter);
    if(geometry)
      addGeometry(pose, *geometry);
  }

  //
  PrimaryObject::createPhysics();

  OpenGLTools::convertTransformation(rotation, translation, transformation);
}

void Compound::addGeometry(const Pose3<>& parentPose, Geometry& geometry)
{
  // compute pose
  Pose3<> geomPose = parentPose;
  if(geometry.translation)
    geomPose.translate(*geometry.translation);
  if(geometry.rotation)
    geomPose.rotate(*geometry.rotation);

  // create geometry
  dGeomID geom = geometry.createGeometry(Simulation::simulation->staticSpace);
  if(geom)
  {
    geometries.push_back(geom);
    dGeomSetData(geom, &geometry);

    // set pose
    dGeomSetPosition(geom, geomPose.translation.x, geomPose.translation.y, geomPose.translation.z);
    dMatrix3 matrix3;
    ODETools::convertMatrix(geomPose.rotation, matrix3);
    dGeomSetRotation(geom, matrix3);
  }

  // handle nested geometries
  for(std::list<PhysicalObject*>::const_iterator iter = geometry.children.begin(), end = geometry.children.end(); iter != end; ++iter)
  {
    Geometry* geometry = dynamic_cast<Geometry*>(*iter);
    if(geometry)
      addGeometry(geomPose, *geometry);
  }
}

void Compound::assembleAppearances() const
{
  glPushMatrix();
  glMultMatrixf(transformation);
  PrimaryObject::assembleAppearances();
  glPopMatrix();
}

void Compound::drawPhysics(unsigned int flags) const
{
  glPushMatrix();
  glMultMatrixf(transformation);
  PrimaryObject::drawPhysics(flags);
  glPopMatrix();
}
