/** 
* @file Simulation/Simulation.h
* Declaration of class Simulation
* @author Colin Graf
*/

#pragma once

#include <QString>
#include <string>
#include <list>
#include <vector>
#include <ode/ode.h>

#ifdef WIN32
#include <unordered_map>
#else
#include <tr1/unordered_map>
#endif

#include "SimRobotCore2.h"
#include "Platform/OffscreenRenderer.h"

class Scene;
class Element;
class CollisionSensor;

/**
* @class Simulation
* A class for managing the simulation
*/
class Simulation : public SimRobotCore2::Simulation2
{
public:
  static Simulation* simulation;

  Scene* scene; /**< The root of the scene graph */
  std::list<Element*> elements; /**< All scene graph elements */

  dWorldID physicalWorld; /**< The physical world */
  dSpaceID rootSpace; /**< The root collision space */
  dSpaceID staticSpace; /**< The collision space for static objects */
  dSpaceID movableSpace; /**< The collision space for movable objects */

  OffscreenRenderer renderer; /**< For rendering OpenGL scenes without a regular window */

  unsigned int currentFrameRate; /**< The current frame rate of the simulation */

  /** Default Constructor. */
  Simulation();

  /** Destructor. */
  virtual ~Simulation();

  /**
  * Loads a file and initializes the simulation
  * @param filename The name of the file
  * @param errors The errors that occured during parsing.
  */
  bool loadFile(const std::string& filename, std::list<std::string>& errors);

  /** Executes one simulation step */
  void doSimulationStep();
  unsigned int simulationStep;
  double simulatedTime;
  unsigned int collisions;
  unsigned int contactPoints;

  /** Registers all objects of the simulation (including children, actuators and sensors) at SimRobot's GUI */
  void registerObjects();
  
  /** Registers a collision sensor with all geometries attached to it. */
  void registerCollisionSensor(CollisionSensor* sensor, const std::list<dGeomID>& geometries);

private:
  QString fullName; /**< The name of the hidden scene graph object that can be used for accessing the API functions */
  dJointGroupID contactGroup; /**< The joint group for temporary contact joints used for collision handling */

  /** Computes the frame rate of simulation */
  void updateFrameRate();
  unsigned int lastFrameRateComputationTime;
  unsigned int lastFrameRateComputationStep;
  std::list<CollisionSensor*> collisionSensors;
  std::tr1::unordered_map<dGeomID, CollisionSensor*> geometriesOfCollisionSensors;
  
  /**
  * Static callback method for handling the collision of two geometries
  * @param simulation The simulation
  * @param geom1 The first geometry object for collision testing
  * @param geom2 The second geometry object for collision testing
  */
  static void staticCollisionCallback(Simulation *simulation, dGeomID geom1, dGeomID geom2);

  /**
  * Static callback method for handling the collision of a static geometry with a movable space
  * @param simulation The simulation
  * @param geom1 The first geometry object for collision testing
  * @param geom2 The second geometry object for collision testing
  */
  static void staticCollisionWithSpaceCallback(Simulation *simulation, dGeomID geom1, dGeomID geom2);

  /**
  * Static callback method for handling the collision of two movable spaces
  * @param simulation The simulation
  * @param geom1 The first geometry object for collision testing
  * @param geom2 The second geometry object for collision testing
  */
  static void staticCollisionSpaceWithSpaceCallback(Simulation *simulation, dGeomID geom1, dGeomID geom2);

  //API
  virtual const QString& getFullName() const {return fullName;}
  virtual double getStepLength() const;
  virtual unsigned int getStep() const {return simulationStep;}
  virtual double getTime() const {return simulatedTime;}
  virtual unsigned int getFrameRate() const {return currentFrameRate;}
};
