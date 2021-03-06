/**
* @file CombinedWorldModelProvider.h
* Declares a class that provides a combined world model
* @author Katharina Gillmann
*/

#ifndef CombinedWorldModelProvider_H
#define CombinedWorldModelProvider_H

#include "Tools/Module/Module.h"
#include "Representations/Modeling/CombinedWorldModel.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/BallHypotheses.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/TeamMateData.h"
#include "Representations/Infrastructure/TeamInfo.h"
#include "Representations/Modeling/RobotsModel.h"
#include "Representations/Modeling/FallDownState.h"
#include "Representations/Sensing/GroundContactState.h"
#include "Representations/Modeling/ObstacleModel.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/Configuration/DamageConfiguration.h"
#include "Tools/Math/Vector2.h"
#include "Tools/Math/Vector3.h"
#include "Tools/RingBuffer.h"

MODULE(CombinedWorldModelProvider)
  REQUIRES(RobotPose)
  REQUIRES(RobotInfo)
  REQUIRES(RobotsModel)
  REQUIRES(BallModel)
  REQUIRES(BallHypotheses)
  REQUIRES(TeamMateData)
  REQUIRES(OpponentTeamInfo)
  REQUIRES(OwnTeamInfo)
  REQUIRES(FieldDimensions)
  REQUIRES(FrameInfo)
  REQUIRES(FallDownState)
  REQUIRES(GroundContactState)
  REQUIRES(GroundTruthBallModel)
  REQUIRES(ObstacleModel)
  REQUIRES(CameraMatrix)
  REQUIRES(DamageConfiguration)
  PROVIDES_WITH_MODIFY_AND_DRAW(CombinedWorldModel)
END_MODULE

/**
 * @class CombinedWorldModelProvider
 * A combined world model
 */
class CombinedWorldModelProvider : public CombinedWorldModelProviderBase
{

public:
  class DetectedRobot
  {
  public:

    GaussianDistribution meanAndCovariance; // includes the position and the uncertainty of the detected robot
    int clusterId; // Id of the Cluster the robot was added to
    vector<DetectedRobot*> measurementsSmallerThanDistance; // pointer to all robots which are closer to the own robot than the declared distance

    DetectedRobot() {} // Constructor
    DetectedRobot(const Vector2<>& robotPosition, const Matrix2x2<>& covariance) : // initially clusterId is set to -1, which means that it was added to no cluster yet
      meanAndCovariance(robotPosition, covariance), clusterId(-1) {}

    bool operator<(const DetectedRobot& other) const  // sorts the detected robots by descending size (>).
    {
      return measurementsSmallerThanDistance.size() > other.measurementsSmallerThanDistance.size();
    }
  };

  class Cluster
  {
  public:
    vector<DetectedRobot*> detectedRobots; // pointer to all robots which belong to the same cluster
  };

  /**
  * Provides the combined world model representation
  */
  void update(CombinedWorldModel& combinedWorldModel);

  /**
  * Initializes some attributes of the module.
  */
  void init();

private:
  /**
  * @class Parameters
  * The parameters of the module
  */
  class Parameters: public Streamable
  {
  private:
    /**
    * The method makes the object streamable.
    * @param in The stream from which the object is read
    * @param out The stream to which the object is written
    */
    virtual void serialize(In* in, Out* out)
    {
      STREAM_REGISTER_BEGIN();
      STREAM(movementFactorBallDisappeared);
      STREAM(movementFactorBallSinceLastSeen);
      STREAM(scalingFactorBallDisappeared);
      STREAM(scalingFactorBallSinceLastSeen);
      STREAM(clusteringDistance);
      STREAM(distanceToTeamMate);
      STREAM(ballModelAge);
      STREAM_REGISTER_FINISH();
    }

  public:
    float movementFactorBallDisappeared;   /**< factor for the movement of the sigmoid function for the ball disappeared weight */
    float movementFactorBallSinceLastSeen; /**< factor for the movement of the sigmoid function for the ball time since last seen weight */
    float scalingFactorBallDisappeared;    /**< factor for the scaling of the sigmoid function for the ball disappeared weight */
    float scalingFactorBallSinceLastSeen;  /**< factor for the scaling of the sigmoid function for the ball time since last seen weight */
    float clusteringDistance;              /**< The distance between obstacles which are added to the same cluster */
    float distanceToTeamMate;              /**< distance of an obstacle to an own teammate */
    int ballModelAge; /**<minimum age of the old used ballModel>*/

    /** Constructor */
    Parameters(): movementFactorBallDisappeared(1000), movementFactorBallSinceLastSeen(5000),
      scalingFactorBallDisappeared(250), scalingFactorBallSinceLastSeen(1000), clusteringDistance(500), distanceToTeamMate(400), ballModelAge(500)
    {
    }
  };

  class ExtendedBallModel : public BallModel
  {
  public:
    float cameraHeight;
    ExtendedBallModel() {}
    ExtendedBallModel(const BallModel& ballModel, float cameraHeight) : BallModel(ballModel), cameraHeight(cameraHeight) {}
  };

  RingBuffer<ExtendedBallModel, 20> ballModelsAllPlayers[TeamMateData::numOfPlayers]; // last x BallModels of each player
  ExtendedBallModel lastValidBallModel[TeamMateData::numOfPlayers]; // last valid BallModels
  bool oldBallModelUsed[TeamMateData::numOfPlayers]; // if old BallModel shall be used

  Parameters parameters; /**< The parameters of this module */
  vector<DetectedRobot> allDetectedRobots; // all detected robots with cluster informations
  vector<Cluster> allCluster; // all found clusters

  BallState getCombinedBallPosition(bool& ballIsValid); // calculates the global ball position
  float computeWeights(const ExtendedBallModel& ballModel, const RobotPose& robotPose, const BallHypotheses& ballHypotheses) const; // computes weight for the global ball position
  void clusterAllDetectedRobots(); // clusters all detected robots
  vector<GaussianDistribution> getPositionOfOpponentRobots(); // calculates the positions and covariance of the opponent robots by using the clusters. All positions inside one cluster are merged by using the last Kalmafilter step.
  void recursiveClustering(DetectedRobot& currentRobot, const int clusterId); // adds robots recursive to a cluster
  bool ownTeamMatesAreMeasured(const Vector2<>& positionOfMeasurment, const vector<Pose2D>& ownTeam, const Vector2<>& ownPosition); // checks if an own team mate is measured with an ultrasonic measurement

};

#endif // CombinedWorldModelProvider_H
