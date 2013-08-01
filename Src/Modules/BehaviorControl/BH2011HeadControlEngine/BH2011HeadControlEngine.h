/**
* @file BH2010HeadControlEngine.h
* @author Colin Graf
*/

#ifndef BH2011HeadControlEngine_H
#define BH2011HeadControlEngine_H

#include "Tools/Module/Module.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/JointData.h"
#include "Representations/Infrastructure/TeamMateData.h"
#include "Representations/BehaviorControl/HeadMotionProposal.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/Perception/GoalPercept.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/BallHypotheses.h"
#include "Representations/MotionControl/HeadJointRequest.h"

MODULE(BH2011HeadControlEngine)
  REQUIRES(FrameInfo)
  REQUIRES(FieldDimensions)
  REQUIRES(RobotDimensions)
  REQUIRES(HeadMotionProposal)
  REQUIRES(CameraMatrix)
  REQUIRES(CameraInfo)
  REQUIRES(FilteredJointData)
  REQUIRES(RobotPose)
  REQUIRES(BallModel)
  REQUIRES(TeamMateData)
  REQUIRES(BallHypotheses)
  REQUIRES(HeadJointRequest)
  REQUIRES(GoalPercept)
  PROVIDES_WITH_MODIFY(HeadMotionRequest)
END_MODULE

class BH2011HeadControlEngine : public BH2011HeadControlEngineBase
{
public:
  BH2011HeadControlEngine();

private:
  /**
  * A collection of parameters for head control engine.
  */
  class Parameters : public Streamable
  {
  public:
    /** Default constructor. */
    Parameters() {}

    float timeFactor; 
    float fieldDistanceFactor;
    float imageDistanceFactor;
    float ballFactorBonus;

    float defaultHeadSpeed;
    //float minMovingSpeed;

  private:
    /**
    * The method makes the object streamable.
    * @param in The stream from which the object is read.
    * @param out The stream to which the object is written. 
    */
    virtual void serialize(In* in, Out* out)
    {  
      STREAM_REGISTER_BEGIN();
      STREAM(timeFactor);
      STREAM(fieldDistanceFactor);
      STREAM(imageDistanceFactor);
      STREAM(ballFactorBonus);
      STREAM(defaultHeadSpeed);
      //STREAM(minMovingSpeed);
      STREAM_REGISTER_FINISH();
    }
  };

  class PointOfInterest
  {
  public:
    Vector2<> pointOnField;
    bool relative;
    bool active;
    unsigned int lastSeenTime;
    Vector2<> pointInImage;
    
    PointOfInterest() {}
    PointOfInterest(int x, int y, bool relative = false) : pointOnField(float(x), float(y)), relative(relative), active(true), lastSeenTime(0) {}
    PointOfInterest(float x, float y, bool relative = false) : pointOnField(x, y), relative(relative), active(true), lastSeenTime(0) {}
  };

  Parameters p;
  PointOfInterest pointsOfInterest[33];
  int lastUsedPointIndex;
  unsigned int lastTargetSwitchTime;
  //unsigned int lastIteration;
  //Vector2<> lastMeasuredPosition;

  void init();

  void update(HeadMotionRequest& headMotionRequest);

  bool relative2image(const Pose3D& cameraMatrixInv, const Vector2<>& relative, Vector2<>& image) const;

};

#endif // BH2010HeadControlEngine_H
