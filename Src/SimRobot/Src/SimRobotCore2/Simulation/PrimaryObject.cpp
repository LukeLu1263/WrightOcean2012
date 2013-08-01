/** 
* @file Simulation/PrimaryObject.h
* Implementation of class PrimaryObject
* @author Colin Graf
*/

#include "Simulation/PrimaryObject.h"
#include "Platform/Assert.h"

void PrimaryObject::addParent(Element& element, bool addDrawing)
{
  ASSERT(!parent);
  parent = dynamic_cast<PrimaryObject*>(&element);
  PhysicalObject* physicalParent = dynamic_cast<PhysicalObject*>(parent);
  physicalParent->children.push_back(this);
  if(addDrawing)
  {
    physicalParent->drawings.push_back(this);
    dynamic_cast<GraphicalObject*>(parent)->drawings.push_back(this);
  }
  SimObject::addParent(element);
}
