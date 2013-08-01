/** 
* @file Simulation/Geometries/BoxGeometry.cpp
* Implementation of class BoxGeometry
* @author Colin Graf
*/

#include <GL/glew.h>

#include "Simulation/Geometries/BoxGeometry.h"

dGeomID BoxGeometry::createGeometry(dSpaceID space)
{
  Geometry::createGeometry(space);
  approxRadius = std::min(std::min(depth, width), height) * 0.5f;
  approxRadiusSqr = approxRadius * approxRadius;
  return dCreateBox(space, depth, width, height);
}

void BoxGeometry::drawPhysics(unsigned int flags) const
{
  glPushMatrix();
  glMultMatrixf(transformation);

  if(flags & SimRobotCore2::Renderer::showPhysics)
  {
    const float lx = depth * 0.5f;
    const float ly = width * 0.5f;
    const float lz = height * 0.5f;

    // sides
    glBegin (GL_TRIANGLE_STRIP);
      glColor4fv(color);
      glNormal3f (-1,0,0);
      glVertex3f (-lx,-ly,-lz);
      glVertex3f (-lx,-ly,lz);
      glVertex3f (-lx,ly,-lz);
      glVertex3f (-lx,ly,lz);
      glNormal3f (0,1,0);
      glVertex3f (lx,ly,-lz);
      glVertex3f (lx,ly,lz);
      glNormal3f (1,0,0);
      glVertex3f (lx,-ly,-lz);
      glVertex3f (lx,-ly,lz);
      glNormal3f (0,-1,0);
      glVertex3f (-lx,-ly,-lz);
      glVertex3f (-lx,-ly,lz);
    glEnd();

    // bottom face
    glBegin (GL_TRIANGLE_FAN);
      glNormal3f (0,0,-1);
      glVertex3f (-lx,-ly,-lz);
      glVertex3f (-lx,ly,-lz);
      glVertex3f (lx,ly,-lz);
      glVertex3f (lx,-ly,-lz);
    glEnd();

    // top face
    glBegin (GL_TRIANGLE_FAN);
      glNormal3f (0,0,1);
      glVertex3f (-lx,-ly,lz);
      glVertex3f (lx,-ly,lz);
      glVertex3f (lx,ly,lz);
      glVertex3f (-lx,ly,lz);
    glEnd();
  }

  PhysicalObject::drawPhysics(flags);
  glPopMatrix();
}
