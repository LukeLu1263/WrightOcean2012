/** 
* @file Simulation/Scene.h
* Declaration of class Scene
* @author Colin Graf
*/

#pragma once

#ifdef WIN32
#include <unordered_map>
#else
#include <tr1/unordered_map>
#endif

#include "Simulation/PrimaryObject.h"
#include "Simulation/Appearances/Appearance.h"
#include "Tools/Texture.h"

class Body;
class Actuator;

/**
* @class Scene
* Class for the root node of the scene graph
*/
class Scene : public PrimaryObject
{
public:
  /**
  * @class Light
  * A scene light definition
  */
  class Light : public Element
  {
  public:
    float diffuseColor[4];
    float ambientColor[4];
    float specularColor[4];
    float position[4];
    float constantAttenuation;
    float linearAttenuation;
    float quadraticAttenuation;
    float spotCutoff; /**< in radian */
    Vector3<> spotDirection;
    float spotExponent;

    /** Default constructor */
    Light();

    /**
    * Registers an element as parent
    * @param element The element to register
    */
    virtual void addParent(Element& element);
  };

  float color[4]; /**< The background (clear color) */
  float stepLength; /**< The length of a simulation step */
  float gravity; /**< The gravity in the simulated world */
  float erp; /**< ODE's erp parameter */
  float cfm; /**< ODE's cfm parameter */
  int contactMode; /**< The default contact mode for contacts between bodies */
  float contactSoftERP;
  float contactSoftCFM;
  bool useQuickSolver; /**< Whether to use ODE's quick solver */
  int quickSolverIterations; /**< The iteration count for ODE's quick solver */
  int quickSolverSkip; /**< Controls how often the normal solver will be used instead of the quick solver */

  Appearance::Surface* defaultSurface; /**< A surface that will be used for drawing physical objects */

  std::list<Body*> bodies; /**< List of bodies without a parent body */
  std::list<Actuator*> actuators; /**< List of actuators that need to do something in every simulation step */
  std::list<Light*> lights; /** List of scene lights */

  /** Default constructor */
  Scene() : contactMode(0), useQuickSolver(false), quickSolverIterations(-1), lastTransformationUpdateStep(0)
  {
    color[0] = color[1] = color[2] = color[3] = 0.f;
    defaultSurface = new Appearance::Surface();
  }

  /** Updates the transformation of movable objects */
  void updateTransformations();
  unsigned int lastTransformationUpdateStep;

  /** Updates all actuators that need to do something for each simulation step */
  void updateActuators();

  /**
  * Prepares the object and the currently selected OpenGL context for drawing the object. 
  * Loads textures and creates display lists. Hence, this function is called for each OpenGL 
  * context the object should be drawn in.
  * @param isShared Whether the OpenGL context has shared display lists and textures
  */
  void createGraphics(bool isShared);

  /** Draws appearance primitives of the object (including children) on the currently selected OpenGL context (as fast as possible) */
  virtual void drawAppearances() const;

  /**
  * Draws physical primitives of the object (including children) on the currently selected OpenGL context
  * @param flags Flags to enable or disable certain features
  */
  virtual void drawPhysics(unsigned int flags) const;

  /**
  * Loads the texture if it is not already loaded
  * @param The file to load the texture from
  * @return The texture or 0 if the texture cannot be loaded
  */
  Texture* loadTexture(const std::string& file);

private:
  std::tr1::unordered_map<std::string, Texture> textures; /**< A storage for loaded textures */

  //API
  virtual const QIcon* getIcon() const;
};
