/** 
* @file Simulation/PrimaryObject.h
* Declaration of class PrimaryObject
* @author Colin Graf
*/

#pragma once

#include "Simulation/GraphicalObject.h"
#include "Simulation/PhysicalObject.h"

/**
* @class PrimaryObject
* Abstract class for scene graph objects with graphical or subordinate graphical representation and physical representation
*/
class PrimaryObject : public GraphicalObject, public PhysicalObject
{
public:
  PrimaryObject* parent; /**< The only parent of the primary object (or \c 0 in case that this is the root object) */

  /** Default constructor */
  PrimaryObject() : parent(0) {}

protected:
  /** 
  * Registers an element as parent
  * @param element The element to register
  * @param addDrawing Whether the object will be drawn relative to its parent
  */
  virtual void addParent(Element& element, bool addDrawing);

  /** 
  * Registers an element as parent
  * @param element The element to register
  */
  virtual void addParent(Element& element) {addParent(element, true);}
};
