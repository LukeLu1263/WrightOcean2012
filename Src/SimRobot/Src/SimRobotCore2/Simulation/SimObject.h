/** 
* @file Simulation/SimObject.h
* Declaration of class SimObject
* @author Colin Graf
*/

#pragma once

#include <QString>
#include <string>
#include <list>

#include "SimRobotCore2.h"
#include "Parser/Element.h"
#include "Tools/Vector3.h"
#include "Tools/Matrix3x3.h"

/**
* @class SimObject
* Abstract class for scene graph objects with a name and a transformation
*/
class SimObject : public Element, public SimRobotCore2::Object
{
public:
  QString fullName; /**< The path name to the object in the scene graph */
  std::string name; /**< The name of the scene graph object (without path) */
  std::list<SimObject*> children; /**< List of subordinate scene graph objects */
  Vector3<>* translation; /**< The initial translational offset relative to the origin of the parent object */
  Matrix3x3<>* rotation; /**< The initial rotational offset relative to the origin of the parent object */
  float transformation[16]; /**< The (updated) offset relative to the origin of the parent object as OpenGL transformation */

  /** Default constructor */
  SimObject();

  /** Destructor */
  virtual ~SimObject();

  /** Registers this object with children, actuators and sensors at SimRobot's GUI */
  virtual void registerObjects();

protected:
  /** 
  * Registers an element as parent
  * @param element The element to register
  */
  virtual void addParent(Element& element);

private:
  // API
  virtual const QString& getFullName() const {return fullName;}
  virtual SimRobot::Widget* createWidget();
  virtual const QIcon* getIcon() const;
  virtual SimRobotCore2::Renderer* createRenderer();
  virtual const float* getPosition() const {return 0;}
  virtual const float* getRotation() const {return 0;}
  virtual void move(const float* pos, const float* rot) {}
  virtual void move(const float* pos) {}
  virtual void resetDynamics() {}
  virtual bool registerDrawing(SimRobotCore2::Controller3DDrawing& drawing) {return false;}
  virtual bool unregisterDrawing(SimRobotCore2::Controller3DDrawing& drawing) {return false;}
};
