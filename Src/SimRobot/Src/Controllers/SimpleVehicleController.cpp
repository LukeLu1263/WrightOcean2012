/**
 * @file SimpleVehicleController
 * 
 * Controller for the demo specified in SimpleVehicle.ros2.
 * A simplistic car is equipped with a laser range finder, 
 * searches a ball and drives towards it.
 *
 * This demo includes:
 * - DepthImageSensor
 * - Hinge joint
 * - Controller3DDrawing
 *
 * @author <A href="mailto:Tim.Laue@dfki.de">Tim Laue</A>
 */


#include <SimRobotCore2.h>
#include <GL/glew.h>
#include <QString>
#include <QVector>
#include <cmath>
#ifdef WIN32
#define M_PI 3.14159265358979323846
#endif

/**
* @class TestDrawing
* An object (a simple sphere) to be drawn by this controller.
*/
class TestDrawing : public SimRobotCore2::Controller3DDrawing
{
  void draw()
  {
    GLUquadricObj* q = gluNewQuadric();
    gluSphere(q, 0.5, 15, 15);
    gluDeleteQuadric(q);
  }
};


/**
* @class SimpleVehicleController
* The controller class for the SimpleVehicle demo
*/
class SimpleVehicleController : public SimRobot::Module
{
private:
  SimRobot::Application&   simRobot;            /** Reference to the SimRobot application */
  SimRobotCore2::Actuator* frontLeftWheel;      /** Access to front left wheel */
  SimRobotCore2::Actuator* frontRightWheel;     /** Access to front right wheel */
  SimRobotCore2::Actuator* backLeftWheel;       /** Access to back left wheel */
  SimRobotCore2::Actuator* backRightWheel;      /** Access to back right wheel */
  SimRobotCore2::Sensor*   distanceSensor;      /** Access to distance sensor */
  bool ballFound;                               /** Flag for behavior */
  float angleToBall;                            /** Angle for behavior */
  float distanceToBall;                         /** Distance for behavior */

  enum VehicleState
  {
    SEARCH_FOR_BALL = 0,
    GO_TO_BALL
  } vehicleState;         /** Different states for the robot behavior */


public:
  /** Constructor */
  SimpleVehicleController(SimRobot::Application& simRobot):simRobot(simRobot)
  {}


  /** Initializes drawing and objects for interfacing with actuators and sensor */
  bool compile()
  {
    // Init 3D drawing, attach it to the ball
    TestDrawing* testDrawing = new TestDrawing();
    SimRobotCore2::Object* ballObj = (SimRobotCore2::Object*)simRobot.resolveObject("SimpleVehicle.Ball", SimRobotCore2::object);
    ballObj->registerDrawing(*testDrawing);

    // Get all necessary actuator and sensor objects
    SimRobotCore2::Object* vehicleObj = (SimRobotCore2::Object*)simRobot.resolveObject("SimpleVehicle.Vehicle", SimRobotCore2::object);
    QVector<QString> parts;
    parts.resize(1);
    parts[0] = "frontLeft.velocity";
    frontLeftWheel = (SimRobotCore2::Actuator*)simRobot.resolveObject(parts, vehicleObj, SimRobotCore2::actuator);
    parts[0] = "frontRight.velocity";
    frontRightWheel = (SimRobotCore2::Actuator*)simRobot.resolveObject(parts, vehicleObj, SimRobotCore2::actuator);
    parts[0] = "backLeft.velocity";
    backLeftWheel = (SimRobotCore2::Actuator*)simRobot.resolveObject(parts, vehicleObj, SimRobotCore2::actuator);
    parts[0] = "backRight.velocity";
    backRightWheel = (SimRobotCore2::Actuator*)simRobot.resolveObject(parts, vehicleObj, SimRobotCore2::actuator);
    parts[0] = "image";
    distanceSensor = (SimRobotCore2::Sensor*)simRobot.resolveObject(parts, vehicleObj, SimRobotCore2::sensor);
 
    // Init behavior member
    vehicleState = SEARCH_FOR_BALL;
    ballFound = false;
    simRobot.setStatusMessage("Initial search for ball.");
    return true;
  }


  /** This function is called in every execution cycle of the simulation*/
  void update()
  {
    detectBall();
    // Oh behave:
    if(vehicleState == SEARCH_FOR_BALL)
    {
      frontLeftWheel->setValue(M_PI/1.5);
      frontRightWheel->setValue(-M_PI/1.5);
      backLeftWheel->setValue(M_PI/1.5);
      backRightWheel->setValue(-M_PI/1.5);
      if(ballFound)
      {
        simRobot.setStatusMessage("Ball detected. Driving to ball.");
        vehicleState = GO_TO_BALL;
      }
    }
    else if(vehicleState == GO_TO_BALL)
    {
      driveToBall();
      if(ballFound == false)
      {
        simRobot.setStatusMessage("Lost ball. Searching ball again.");
        vehicleState = SEARCH_FOR_BALL;
      }
    }
  }


  /** Tries to find the ball in the laser range finder data. Sets some members. */
  void detectBall()
  {
    ballFound = false;
    // Find closest measurement:
    float* distanceData = (float*)distanceSensor->getValue().floatArray;
    float minDist = distanceData[0];
    int minDistIdx = 0;
    const int numOfDist = distanceSensor->getDimensions()[0];
    for(int i=1; i<numOfDist; ++i)
    {
      if(distanceData[i] < minDist)
      {
        minDist = distanceData[i];
        minDistIdx = i;
      }
    }
    // Compute relative ball position (if anything has been measured):
    float sensorMinDist, sensorMaxDist;
    distanceSensor->getMinAndMax(sensorMinDist, sensorMaxDist);
    if(minDist/sensorMaxDist < 0.9)
    {
      ballFound = true;
      const float openingAngle = M_PI * 2.0f / 3.0f;
      angleToBall = openingAngle/2.0 - (minDistIdx+0.5)*(openingAngle/numOfDist);
      distanceToBall = minDist;
    }
  }


  /** Drives to the ball. */
  void driveToBall()
  {
    if(angleToBall < -0.1)
    {
      frontLeftWheel->setValue(M_PI/1.5);
      frontRightWheel->setValue(-M_PI/1.5);
      backLeftWheel->setValue(M_PI/1.5);
      backRightWheel->setValue(-M_PI/1.5);
    }
    else if(angleToBall > 0.1)
    {
      frontLeftWheel->setValue(-M_PI/1.5);
      frontRightWheel->setValue(M_PI/1.5);
      backLeftWheel->setValue(-M_PI/1.5);
      backRightWheel->setValue(M_PI/1.5);
    }
    else //(ball is in front of robot)
    {
      frontLeftWheel->setValue(M_PI/1.5);
      frontRightWheel->setValue(M_PI/1.5);
      backLeftWheel->setValue(M_PI/1.5);
      backRightWheel->setValue(M_PI/1.5);
    }
  }
};


extern "C" DLL_EXPORT SimRobot::Module* createModule(SimRobot::Application& simRobot)
{
  return new SimpleVehicleController(simRobot);
}
