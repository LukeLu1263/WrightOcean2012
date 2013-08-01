/**
* @file Simulation/Simulation.cpp
* Implementation of class Simulation
* @author Colin Graf
*/

#include <cmath>
#include <algorithm>

#include "Platform/Assert.h"
#include "Platform/System.h"
#include "Simulation/Simulation.h"
#include "Simulation/Scene.h"
#include "Simulation/Body.h"
#include "Simulation/Geometries/Geometry.h"
#include "Simulation/Sensors/Bumper.h"
#include "Parser/Parser.h"
#include "Tools/ODETools.h"
#include "CoreModule.h"

Simulation* Simulation::simulation = 0;

Simulation::Simulation() : scene(0), physicalWorld(0), rootSpace(0), staticSpace(0), movableSpace(0),
  currentFrameRate(0),
  simulationStep(0), simulatedTime(0), collisions(0), contactPoints(0),
  contactGroup(0),
  lastFrameRateComputationTime(0), lastFrameRateComputationStep(0)
{
  ASSERT(simulation == 0);
  simulation = this;
}

Simulation::~Simulation()
{
  for(std::list<Element*>::const_iterator iter = elements.begin(), end = elements.end(); iter != end; ++iter)
    delete *iter;

  if(contactGroup)
    dJointGroupDestroy(contactGroup);
  if(rootSpace)
    dSpaceDestroy(rootSpace);
  if(physicalWorld)
  {
    dWorldDestroy(physicalWorld);
    dCloseODE();
  }

  ASSERT(simulation == this);
  simulation = 0;
}

bool Simulation::loadFile(const std::string& filename, std::list<std::string>& errors)
{
  ASSERT(scene == 0);

  Parser parser;
  if(!parser.parse(filename, errors))
    return false;

  ASSERT(scene);

  dInitODE();
  physicalWorld = dWorldCreate();
  rootSpace = dHashSpaceCreate(0);
  staticSpace = dHashSpaceCreate(rootSpace);
  movableSpace = dHashSpaceCreate(rootSpace);
  contactGroup = dJointGroupCreate(0);

  dWorldSetGravity(physicalWorld, 0, 0, scene->gravity);
  if(scene->erp != -1.f)
    dWorldSetERP(physicalWorld, scene->erp);
  if(scene->cfm != -1.f)
    dWorldSetCFM(physicalWorld, scene->cfm);
  if(scene->quickSolverIterations != -1)
    dWorldSetQuickStepNumIterations(physicalWorld, scene->quickSolverIterations);

  scene->createPhysics();

  renderer.init();

  return true;
}

void Simulation::doSimulationStep()
{
  ++simulationStep;
  simulatedTime += scene->stepLength;

  scene->updateActuators();

  collisions = contactPoints = 0;
  for(std::list<CollisionSensor*>::const_iterator i = collisionSensors.begin(); i != collisionSensors.end(); ++i)
    (*i)->set(false);

  dSpaceCollide2((dGeomID)staticSpace, (dGeomID)movableSpace, this, (dNearCallback*)&staticCollisionWithSpaceCallback);
  dSpaceCollide(movableSpace, this, (dNearCallback*)&staticCollisionSpaceWithSpaceCallback);

  if(scene->useQuickSolver && (simulationStep % scene->quickSolverSkip) == 0)
    dWorldQuickStep(physicalWorld, scene->stepLength);
  else
    dWorldStep(physicalWorld, scene->stepLength);
  dJointGroupEmpty(contactGroup);

  updateFrameRate();
}

void Simulation::staticCollisionWithSpaceCallback(Simulation* simulation, dGeomID geomId1, dGeomID geomId2)
{
  ASSERT(!dGeomIsSpace(geomId1));
  ASSERT(dGeomIsSpace(geomId2));
  dSpaceCollide2(geomId1, geomId2, simulation, (dNearCallback*)&staticCollisionCallback);
}

void Simulation::staticCollisionSpaceWithSpaceCallback(Simulation* simulation, dGeomID geomId1, dGeomID geomId2)
{
  ASSERT(dGeomIsSpace(geomId1));
  ASSERT(dGeomIsSpace(geomId2));
  dSpaceCollide2(geomId1, geomId2, simulation, (dNearCallback*)&staticCollisionCallback);
}

void Simulation::staticCollisionCallback(Simulation* simulation, dGeomID geomId1, dGeomID geomId2)
{
  ASSERT(!dGeomIsSpace(geomId1));
  ASSERT(!dGeomIsSpace(geomId2));

  dBodyID bodyId1 = dGeomGetBody(geomId1);
  dBodyID bodyId2 = dGeomGetBody(geomId2);
  ASSERT(bodyId1 || bodyId2);

#ifndef NDEBUG
  Body* body1 = bodyId1 ? (Body*)dBodyGetData(bodyId1) : 0;
  Body* body2 = bodyId2 ? (Body*)dBodyGetData(bodyId2) : 0;
#endif

  //if(body1 && body2 && body1->rootBody == body2->rootBody)
    //return; // avoid self collisions of complex movable bodies
  ASSERT(!body1 || !body2 || body1->rootBody != body2->rootBody);

  dContact contact[32];
  int collisions = dCollide(geomId1, geomId2, 32, &contact[0].geom, sizeof(dContact));
  if(collisions <= 0)
    return;

  Geometry* geometry1 = (Geometry*)dGeomGetData(geomId1);
  Geometry* geometry2 = (Geometry*)dGeomGetData(geomId2);

  float friction = 1.f;
  if(geometry1->material && geometry2->material)
  {
    if(!geometry1->material->getFriction(*geometry2->material, friction))
      friction = 1.f;

    float rollingFriction;
    if(bodyId1)
      switch(dGeomGetClass(geomId1))
      {
      case dSphereClass:
      case dCCylinderClass:
      case dCylinderClass:
        if(geometry1->material->getRollingFriction(*geometry2->material, rollingFriction))
        {
          Vector3<> angularVel;
          ODETools::convertVector(dBodyGetAngularVel(bodyId1), angularVel);
          angularVel -= Vector3<>(angularVel).normalize(std::min(angularVel.abs(), std::abs(simulation->scene->gravity) * rollingFriction * simulation->scene->stepLength));
          dBodySetAngularVel(bodyId1, angularVel.x, angularVel.y, angularVel.z);
        }
        break;
      }
    if(bodyId2)
      switch(dGeomGetClass(geomId2))
      {
      case dSphereClass:
      case dCCylinderClass:
      case dCylinderClass:
        if(geometry2->material->getRollingFriction(*geometry1->material, rollingFriction))
        {
          Vector3<> angularVel;
          ODETools::convertVector(dBodyGetAngularVel(bodyId2), angularVel);
          angularVel -= Vector3<>(angularVel).normalize(std::min(angularVel.abs(), std::abs(simulation->scene->gravity) * rollingFriction * simulation->scene->stepLength));
          dBodySetAngularVel(bodyId2, angularVel.x, angularVel.y, angularVel.z);
        }
        break;
      }
  }

  std::tr1::unordered_map<dGeomID, CollisionSensor*>::const_iterator i = simulation->geometriesOfCollisionSensors.find(geomId1);
  if(i != simulation->geometriesOfCollisionSensors.end())
    i->second->set(true);
  i = simulation->geometriesOfCollisionSensors.find(geomId2);
  if(i != simulation->geometriesOfCollisionSensors.end())
    i->second->set(true);

  for (int i = 0; i < collisions; ++i)
  {
    contact[i].surface.mode = simulation->scene->contactMode | dContactApprox1;
    contact[i].surface.mu = friction;

    /*
    contact[i].surface.bounce = 0.f;
    contact[i].surface.bounce_vel = 0.001f;
    contact[i].surface.slip1 = 0.f;
    contact[i].surface.slip2 = 0.f;
    */
    contact[i].surface.soft_erp = simulation->scene->contactSoftERP;
    contact[i].surface.soft_cfm = simulation->scene->contactSoftCFM;


    dJointID c = dJointCreateContact(simulation->physicalWorld, simulation->contactGroup, &contact[i]);
    dJointAttach(c, dGeomGetBody(contact[i].geom.g1), dGeomGetBody(contact[i].geom.g2));
  }
  ++simulation->collisions;
  simulation->contactPoints += collisions;


}

void Simulation::updateFrameRate()
{
  unsigned int currentTime = System::getTime();
  unsigned int timeDiff = currentTime - lastFrameRateComputationTime;
  //Only update frame rate once in two seconds
  if(timeDiff > 2000)
  {
    float frameRate = float(simulationStep - lastFrameRateComputationStep) / (float(timeDiff) * 0.001f);
    currentFrameRate = int(frameRate + 0.5f);
    lastFrameRateComputationTime = currentTime;
    lastFrameRateComputationStep = simulationStep;
  }
}

void Simulation::registerObjects()
{
  scene->fullName = scene->name.c_str();
  CoreModule::application->registerObject(*CoreModule::module, *scene, 0);
  scene->registerObjects();

  fullName = "Simulation2";
  CoreModule::application->registerObject(*CoreModule::module, *this, 0, SimRobot::Flag::hidden);
}

void Simulation::registerCollisionSensor(CollisionSensor* sensor, const std::list<dGeomID>& geometries)
{
  ASSERT(std::find(collisionSensors.begin(), collisionSensors.end(), sensor) == collisionSensors.end());
  collisionSensors.push_back(sensor);
  for(std::list<dGeomID>::const_iterator i = geometries.begin(); i != geometries.end(); ++i)
    geometriesOfCollisionSensors[*i] = sensor;
}

double Simulation::getStepLength() const
{
  return scene->stepLength;
}

