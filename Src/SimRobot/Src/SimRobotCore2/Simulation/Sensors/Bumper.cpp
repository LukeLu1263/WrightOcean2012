/** 
* @file Simulation/Sensors/Bumper.cpp
* Implementation of class Bumper
* @author Thomas Röfer
*/

#include "Simulation/Sensors/Bumper.h"
#include "Simulation/Body.h"
#include "CoreModule.h"

Bumper::Bumper()
{
  sensor.sensorType = SimRobotCore2::Sensor::boolSensor;
}

void Bumper::createPhysics()
{
  Body::createPhysics();
  Simulation::simulation->registerCollisionSensor(&sensor, geometries);
}

void Bumper::registerObjects()
{
  Body::registerObjects();
  sensor.fullName = fullName + ".contact";
  CoreModule::application->registerObject(*CoreModule::module, sensor, this);
}
