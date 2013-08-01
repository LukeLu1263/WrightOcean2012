/** 
* @file Simulation/GraphicalObject.cpp
* Implementation of class GraphicalObject
* @author Colin Graf
*/

#include <GL/glew.h>

#include "Simulation/Simulation.h"
#include "Simulation/Scene.h"
#include "Simulation/GraphicalObject.h"
#include "Platform/Assert.h"

void GraphicalObject::createGraphics()
{
  ++initializedContexts;
  for(std::list<GraphicalObject*>::const_iterator iter = drawings.begin(), end = drawings.end(); iter != end; ++iter)
  {
    GraphicalObject* graphicalObject = *iter;
    if(graphicalObject->initializedContexts != initializedContexts)
    {
      graphicalObject->createGraphics();
      graphicalObject->initializedContexts = initializedContexts;
    }
  }

  // create display list
  unsigned int listId = glGenLists(1);
  ASSERT(listId > 0);
  ASSERT(this->listId == 0 || this->listId == listId);
  this->listId = listId;
  glNewList(listId, GL_COMPILE);
    assembleAppearances();
  glEndList();
}

void GraphicalObject::drawAppearances() const
{
  ASSERT(listId);
  glCallList(listId);
}

void GraphicalObject::assembleAppearances() const
{
  for(std::list<GraphicalObject*>::const_iterator iter = drawings.begin(), end = drawings.end(); iter != end; ++iter)
    (*iter)->drawAppearances();
}

void GraphicalObject::addParent(Element& element)
{
  dynamic_cast<GraphicalObject*>(&element)->drawings.push_back(this);
}
