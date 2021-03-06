/** 
* @file Simulation/Appearances/CylinderAppearance.cpp
* Implementation of class CylinderAppearance
* @author Colin Graf
*/

#include <GL/glew.h>

#include "Simulation/Appearances/CylinderAppearance.h"

void CylinderAppearance::assembleAppearances() const
{
  glPushMatrix();
  glMultMatrixf(transformation);

  surface->set();

  GLUquadricObj* q = gluNewQuadric();
  glTranslated(0.f, 0.f, height * -0.5f);
  gluCylinder(q, radius, radius, height, 16, 1);
  glRotatef(180.0f, 0.0f, 1.0f, 0.0f);
  gluDisk(q, 0, radius, 16, 1);
  glRotatef(180.0f, 0.0f, 1.0f, 0.0f);
  glTranslated(0,0,height);
  gluDisk(q, 0, radius, 16, 1);
  gluDeleteQuadric(q);

  surface->unset();

  GraphicalObject::assembleAppearances();
  glPopMatrix();
}
