/**
 * @file FactoryController
 * 
 * Controller for the simple factory demo specified in Factory.ros2.
 * Three distance sensors determine the height of a box. Afterwards,
 * the boxes is pushed to the right trap door which senses contact to
 * a box, opens, and lets the box pass.
 *
 * This demo includes:
 * - SingleDistanceSensor
 * - Bumper sensor
 * - Slider joint
 * - Hinge joint
 *
 * @author <A href="mailto:Tim.Laue@dfki.de">Tim Laue</A>
 * @author Kai Spiess
 */

#include <SimRobotCore2.h>
#include <QString>
#include <QVector>
#include <cmath>
#ifdef WIN32
#include <windows.h>
#define M_PI 3.14159265358979323846
#endif
/**
 *  The controller class for the factory demo.
 */
class FactoryController : public SimRobot::Module
{
private:
  SimRobot::Application&      simRobot;        /** Reference to the SimRobot application */
  SimRobotCore2::Sensor*      distanceSensor1; /**< Access to the first distance sensor **/
  SimRobotCore2::Sensor*      distanceSensor2; /**< Access to the second distance sensor **/
  SimRobotCore2::Sensor*      distanceSensor3; /**< Access to the third distance sensor **/
  SimRobotCore2::Actuator*    sliderActuator;  /**< Access for controlling the slider joint */
  SimRobotCore2::Sensor*      sliderSensor;    /**< Access for measuring the slider joint's position */
  SimRobotCore2::Sensor*      trapDoor1Bumper; /**< Access to the first trap door's collision detector**/
  SimRobotCore2::Sensor*      trapDoor2Bumper; /**< Access to the second trap door's collision detector **/
  SimRobotCore2::Sensor*      trapDoor3Bumper; /**< Access to the third trap door's collision detector **/
  SimRobotCore2::Actuator*    trapDoor1Hinge;  /**< Access to the first trap door's hinge actuator */
  SimRobotCore2::Actuator*    trapDoor2Hinge;  /**< Access to the second trap door's hinge actuator */
  SimRobotCore2::Actuator*    trapDoor3Hinge;  /**< Access to the third trap door's hinge actuator */
  SimRobotCore2::Simulation2* simPort;         /**< Access to some general simulation information */

  enum FactoryState
  {
    MEASURING=0,
    PUSHING_BLOCK1,
    PUSHING_BLOCK2,
    PUSHING_BLOCK3,
    RETURNING
  };

  FactoryState currentState;          /**< Current operational state of simple factory */
  FactoryState nextState;             /**< Next operational state of simple factory */
  float sliderPositions[RETURNING];   /**< Positions for the slider to go to */
  float startOfWaitingTime;           /**< A time stamp for delaying some actions */

public:
   /** Constructor */
  FactoryController(SimRobot::Application& simRobot):simRobot(simRobot)
  {}

  /** Initialization of sensor and actuator ports, states, and positions*/
  bool compile()
  {
    // Get all necessary actuator and sensor objects
    SimRobotCore2::Object* rootObj = (SimRobotCore2::Object*)simRobot.resolveObject("Factory", SimRobotCore2::object);
    QVector<QString> parts;
    parts.resize(1);
    parts[0] = "distance1.distance";
    distanceSensor1 = (SimRobotCore2::Sensor*)simRobot.resolveObject(parts, rootObj, SimRobotCore2::sensor);
    parts[0] = "distance2.distance";
    distanceSensor2 = (SimRobotCore2::Sensor*)simRobot.resolveObject(parts, rootObj, SimRobotCore2::sensor);
    parts[0] = "distance3.distance";
    distanceSensor3 = (SimRobotCore2::Sensor*)simRobot.resolveObject(parts, rootObj, SimRobotCore2::sensor);
    parts[0] = "sliderJoint.position";
    sliderActuator = (SimRobotCore2::Actuator*)simRobot.resolveObject(parts, rootObj, SimRobotCore2::actuator);
    sliderSensor = (SimRobotCore2::Sensor*)simRobot.resolveObject(parts, rootObj, SimRobotCore2::sensor);
    parts[0] = "trapDoor1Sensor.contact";
    trapDoor1Bumper = (SimRobotCore2::Sensor*)simRobot.resolveObject(parts, rootObj, SimRobotCore2::sensor);
    parts[0] = "trapDoor2Sensor.contact";
    trapDoor2Bumper = (SimRobotCore2::Sensor*)simRobot.resolveObject(parts, rootObj, SimRobotCore2::sensor);
    parts[0] = "trapDoor3Sensor.contact";
    trapDoor3Bumper = (SimRobotCore2::Sensor*)simRobot.resolveObject(parts, rootObj, SimRobotCore2::sensor);
    parts[0] = "trapDoor1Hinge.position";
    trapDoor1Hinge = (SimRobotCore2::Actuator*)simRobot.resolveObject(parts, rootObj, SimRobotCore2::actuator);
    parts[0] = "trapDoor2Hinge.position";
    trapDoor2Hinge = (SimRobotCore2::Actuator*)simRobot.resolveObject(parts, rootObj, SimRobotCore2::actuator);
    parts[0] = "trapDoor3Hinge.position";
    trapDoor3Hinge = (SimRobotCore2::Actuator*)simRobot.resolveObject(parts, rootObj, SimRobotCore2::actuator);
    simPort = (SimRobotCore2::Simulation2*)simRobot.resolveObject("Simulation2", SimRobotCore2::simulation);
    currentState = MEASURING;
    nextState = MEASURING;
    startOfWaitingTime = 0.0;
    // Slider target positions:
    sliderPositions[MEASURING] = 0.02;
    sliderPositions[PUSHING_BLOCK1] = 0.35;
    sliderPositions[PUSHING_BLOCK2] = 0.60;
    sliderPositions[PUSHING_BLOCK3] = 0.90;
    return true;
  }


  /** This function becomes called in every execution cycle of the simulation*/
  void update()
  {
    /** Always check for open trap doors and close them */
    bool trapDoor1State = trapDoor1Bumper->getValue().boolValue;
    bool trapDoor2State = trapDoor2Bumper->getValue().boolValue;
    bool trapDoor3State = trapDoor3Bumper->getValue().boolValue;
     
    /** Waiting for box and measuring it */
    if(currentState == MEASURING)
    {
      // Get distance measurements of all three sensors:
      float dist1 = distanceSensor1->getValue().floatValue;
      float dist2 = distanceSensor2->getValue().floatValue;
      float dist3 = distanceSensor3->getValue().floatValue;
      // Determine box
      if(dist3 < 0.40)
        nextState = PUSHING_BLOCK3;
      else if(dist2 < 0.40)
        nextState = PUSHING_BLOCK2;
      else if(dist1 < 0.40)
        nextState = PUSHING_BLOCK1;
      // Wait some cycles before changing to pushing mode since slider crosses sensor
      if(nextState && startOfWaitingTime == 0)
        startOfWaitingTime = simPort->getTime();
      if(startOfWaitingTime != 0 && nextState == 0)
        startOfWaitingTime = 0;
      if((startOfWaitingTime != 0) && (simPort->getTime() - startOfWaitingTime > 3.5))
      {
        simRobot.setStatusMessage("Pushing block.");
        currentState = nextState;
        startOfWaitingTime = 0.0;
      }
      sliderActuator->setValue(sliderPositions[MEASURING]);
    }
    /** Pushing the box number 1 */
    else if(currentState == PUSHING_BLOCK1)
    {
      sliderActuator->setValue(sliderPositions[PUSHING_BLOCK1]);
      if(trapDoor1State)
      {
        trapDoor1Hinge->setValue(-M_PI/3.0);
        if(startOfWaitingTime == 0.0)
          startOfWaitingTime = simPort->getTime();
      }
      if((startOfWaitingTime != 0.0) && (simPort->getTime() - startOfWaitingTime > 4.0))
      {
        trapDoor1Hinge->setValue(0);
        simRobot.setStatusMessage("Pushing small block finished. Slider returns to base.");
        currentState = RETURNING;
      }
    }
    /** Pushing the box number 2 */
    else if(currentState == PUSHING_BLOCK2)
    {
      sliderActuator->setValue(sliderPositions[PUSHING_BLOCK2]);
      if(trapDoor2State)
      {
        trapDoor2Hinge->setValue(-M_PI/3.0);
        if(startOfWaitingTime == 0.0)
          startOfWaitingTime = simPort->getTime();
      }
      if((startOfWaitingTime != 0.0) && (simPort->getTime() - startOfWaitingTime > 4.0))
      {
        trapDoor2Hinge->setValue(0);
        simRobot.setStatusMessage("Pushing medium block finished. Slider returns to base.");
        currentState = RETURNING;
      }
    }
    /** Pushing the box number 3 */
    else if(currentState == PUSHING_BLOCK3)
    {
      sliderActuator->setValue(sliderPositions[PUSHING_BLOCK3]);
      if(trapDoor3State)
      {
        trapDoor3Hinge->setValue(-M_PI/3.0);
        if(startOfWaitingTime == 0.0)
          startOfWaitingTime = simPort->getTime();
      }
      if((startOfWaitingTime != 0.0) && (simPort->getTime() - startOfWaitingTime > 4.0))
      {
        trapDoor3Hinge->setValue(0);
        simRobot.setStatusMessage("Pushing large block finished. Slider returns to base.");
        currentState = RETURNING;
      }
    }
    /** Return to start */
    else if(currentState == RETURNING)
    {
      sliderActuator->setValue(sliderPositions[MEASURING]);
      float currentSliderPosition = sliderSensor->getValue().floatValue;
      if(fabs(currentSliderPosition - sliderPositions[MEASURING]) < 0.01)
      {
        simRobot.setStatusMessage("Slider reached base. Waiting for measurement.");
        currentState = nextState = MEASURING;
      }
    }
#ifdef WIN32
    Sleep(10);
#else
    usleep(10000);
#endif
  };
};


extern "C" DLL_EXPORT SimRobot::Module* createModule(SimRobot::Application& simRobot)
{
  return new FactoryController(simRobot);
}
