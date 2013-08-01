/** 
* @file Simulation/PhysicalObject.cpp
* Implementation of class PhysicalObject
* @author Colin Graf
*/

#include <GL/glew.h>

#include "Simulation/Simulation.h"
#include "Simulation/Scene.h"
#include "Simulation/PhysicalObject.h"

void PhysicalObject::createPhysics()
{
  for(std::list<PhysicalObject*>::const_iterator iter = children.begin(), end = children.end(); iter != end; ++iter)
  {
    // compute pose of child object
    PhysicalObject* object = *iter;
    object->pose = pose;
    if(object->translation)
      object->pose.translate(*object->translation);
    if(object->rotation)
      object->pose.rotate(*object->rotation);

    //
    object->createPhysics();
  }
}

void PhysicalObject::drawPhysics(unsigned int flags) const
{
  if(flags & SimRobotCore2::Renderer::showControllerDrawings)
    for(std::list<SimRobotCore2::Controller3DDrawing*>::const_iterator iter = controllerDrawings.begin(), end = controllerDrawings.end(); iter != end; ++iter)
      (*iter)->draw();
  for(std::list<PhysicalObject*>::const_iterator iter = drawings.begin(), end = drawings.end(); iter != end; ++iter)
    (*iter)->drawPhysics(flags);
}

void PhysicalObject::addParent(Element& element)
{
  PhysicalObject* physicalParent = dynamic_cast<PhysicalObject*>(&element);
  physicalParent->children.push_back(this);
  physicalParent->drawings.push_back(this);
  SimObject::addParent(element);
}

bool PhysicalObject::registerDrawing(SimRobotCore2::Controller3DDrawing& drawing)
{
  controllerDrawings.push_back(&drawing);
  return true;
}

bool PhysicalObject::unregisterDrawing(SimRobotCore2::Controller3DDrawing& drawing)
{
  for(std::list<SimRobotCore2::Controller3DDrawing*>::iterator iter = controllerDrawings.begin(), end = controllerDrawings.end(); iter != end; ++iter)
    if(*iter == &drawing)
    {
      controllerDrawings.erase(iter);
      return true;
    }
  return false;
}

