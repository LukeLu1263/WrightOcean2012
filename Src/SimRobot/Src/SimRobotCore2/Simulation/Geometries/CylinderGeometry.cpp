/** 
* @file Simulation/Geometries/CylinderGeometry.cpp
* Implementation of class CylinderGeometry
* @author Colin Graf
*/

#include <GL/glew.h>

#include "Simulation/Geometries/CylinderGeometry.h"

dGeomID CylinderGeometry::createGeometry(dSpaceID space)
{
  Geometry::createGeometry(space);
  approxRadius = radius;
  approxRadiusSqr = approxRadius * approxRadius;
  return dCreateCylinder(space, radius, height);
}

void CylinderGeometry::drawPhysics(unsigned int flags) const
{
  glPushMatrix();
  glMultMatrixf(transformation);

  if(flags & SimRobotCore2::Renderer::showPhysics)
  {
    glColor4fv(color);
    GLUquadricObj* q = gluNewQuadric();
    glTranslated(0.f, 0.f, height * -0.5f);
    gluCylinder(q, radius, radius, height, 16, 1);
    glRotatef(180.0f, 0.0f, 1.0f, 0.0f);
    gluDisk(q, 0, radius, 16, 1);
    glRotatef(180.0f, 0.0f, 1.0f, 0.0f);
    glTranslated(0,0,height);
    gluDisk(q, 0, radius, 16, 1);
    gluDeleteQuadric(q);
  }

  PhysicalObject::drawPhysics(flags);
  glPopMatrix();
}
