/** 
* @file Simulation/Appearances/BoxAppearance.cpp
* Implementation of class BoxAppearance
* @author Colin Graf
*/

#include <GL/glew.h>

#include "Simulation/Appearances/BoxAppearance.h"

void BoxAppearance::assembleAppearances() const
{
  glPushMatrix();
  glMultMatrixf(transformation);

  surface->set();

  float lx = depth * 0.5f;
  float ly = width * 0.5f;
  float lz = height * 0.5f;

  // sides
  glBegin (GL_TRIANGLE_STRIP);
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

  surface->unset();

  GraphicalObject::assembleAppearances();
  glPopMatrix();
}
