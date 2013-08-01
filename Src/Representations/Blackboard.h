/**
* @file Blackboard.h
* Declaration of a class representing the blackboard.
* @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
*/

#pragma once

#include <cstddef>
#include "Platform/SystemCall.h"

// Declare prototypes of all representations here:

// Infrastructure
class JointData;
class JointRequest;
class SensorData;
class KeyStates;
class LEDRequest;
class Image;
class CameraInfo;
class FrameInfo;
class CognitionFrameInfo;
class RobotInfo;
class OwnTeamInfo;
class OpponentTeamInfo;
class GameInfo;
class SoundRequest;
class SoundOutput;
class TeamMateData;
class MotionRobotHealth;
class RobotHealth;
class BoardInfo;

// Configuration
class ColorTable64;
class CameraSettings;
class FieldDimensions;
class RobotDimensions;
class JointCalibration;
class SensorCalibration;
class CameraCalibration;
class BehaviorConfiguration;
class MassCalibration;
class HardnessSettings;
class DamageConfiguration;

// Perception
class CameraMatrix;
class CameraMatrixOther;
class CameraMatrixPrev;
class RobotCameraMatrix;
class RobotCameraMatrixOther;
class RobotCameraMatrixPrev;
class ImageCoordinateSystem;
class BallSpots;
class LineSpots;
class BallPercept;
class LinePercept;
class RegionPercept;
class GoalPercept;
class GroundContactState;
class BodyContour;
class TeamMarkerSpots;
class RobotPercept;
class ImageInfo;
class ImageRequest;

// Modeling
class ArmContactModel;
class FallDownState;
class BallAfterKickPose;
class BallHypotheses;
class BallModel;
class CombinedWorldModel;
class GroundTruthBallModel;
class ObstacleModel;
class USObstacleGrid;
class RobotPose;
class RobotPoseInfo;
class PotentialRobotPose;
class GroundTruthRobotPose;
class RobotPoseHypotheses;
class RobotsModel;
class GroundTruthRobotsModel;
class FreePartOfOpponentGoalModel;
class SSLVisionData;
class GroundTruthResult;
class FieldCoverage;
class GlobalFieldCoverage;

// BehaviorControl
class BehaviorControlOutput;
class KickInfo;
class BehaviorLEDRequest;
class HeadMotionProposal;

// Sensing
class FilteredJointData;
class FilteredJointDataPrev;
class FilteredSensorData;
class InertiaSensorData;
class InspectedInertiaSensorData;
class OrientationData;
class GroundTruthOrientationData;
class TorsoMatrix;
class RobotModel;

// MotionControl
class OdometryData;
class GroundTruthOdometryData;
class MotionRequest;
class HeadMotionRequest;
class HeadAngleRequest;
class HeadJointRequest;
class MotionSelection;
class SpecialActionsOutput;
class WalkingEngineOutput;
class WalkingEngineStandOutput;
class BikeEngineOutput;
class MotionInfo;
class UnstableJointRequest;

// Debugging
class XabslInfo;

// friends
class Process;
class Cognition;
class Motion;
class Framework;

/**
* @class Blackboard
* The class represents the blackboard that contains all representation.
* Note: The blackboard only contains references to the objects as attributes.
* The actual representations are constructed on the heap, because many copies of
* of the blackboard exist but only a single set of the representations shared
* by all instances.
*/
class Blackboard
{
protected:
  // Add all representations as constant references here:
  // Infrastructure
  const JointData& theJointData;
  const JointRequest& theJointRequest;
  const SensorData& theSensorData;
  const KeyStates& theKeyStates;
  const LEDRequest& theLEDRequest;
  const Image& theImage;
  const CameraInfo& theCameraInfo;
  const FrameInfo& theFrameInfo;
  const CognitionFrameInfo& theCognitionFrameInfo;
  const RobotInfo& theRobotInfo;
  const OwnTeamInfo& theOwnTeamInfo;
  const OpponentTeamInfo& theOpponentTeamInfo;
  const GameInfo& theGameInfo;
  const SoundRequest& theSoundRequest;
  const SoundOutput& theSoundOutput;
  const TeamMateData& theTeamMateData;
  const MotionRobotHealth& theMotionRobotHealth;
  const RobotHealth& theRobotHealth;
  const BoardInfo& theBoardInfo;

  // Configuration
  const ColorTable64& theColorTable64;
  const CameraSettings& theCameraSettings;
  const FieldDimensions& theFieldDimensions;
  const RobotDimensions& theRobotDimensions;
  const JointCalibration& theJointCalibration;
  const SensorCalibration& theSensorCalibration;
  const CameraCalibration& theCameraCalibration;
  const BehaviorConfiguration& theBehaviorConfiguration;
  const MassCalibration& theMassCalibration;
  const HardnessSettings& theHardnessSettings;
  const DamageConfiguration& theDamageConfiguration;

  // Perception
  const CameraMatrix& theCameraMatrix;
  const CameraMatrixOther& theCameraMatrixOther;
  const CameraMatrixPrev& theCameraMatrixPrev;
  const RobotCameraMatrix& theRobotCameraMatrix;
  const RobotCameraMatrixOther& theRobotCameraMatrixOther;
  const RobotCameraMatrixPrev& theRobotCameraMatrixPrev;
  const ImageCoordinateSystem& theImageCoordinateSystem;
  const BallSpots& theBallSpots;
  const LineSpots& theLineSpots;
  const BallPercept& theBallPercept;
  const LinePercept& theLinePercept;
  const RegionPercept& theRegionPercept;
  const GoalPercept& theGoalPercept;
  const GroundContactState& theGroundContactState;
  const BodyContour& theBodyContour;
  const TeamMarkerSpots& theTeamMarkerSpots;
  const RobotPercept& theRobotPercept;
  const ImageInfo& theImageInfo;
  const ImageRequest& theImageRequest;

  // Modeling
  const ArmContactModel& theArmContactModel;
  const FallDownState& theFallDownState;
  const BallAfterKickPose& theBallAfterKickPose;
  const BallHypotheses& theBallHypotheses;
  const BallModel& theBallModel;
  const CombinedWorldModel& theCombinedWorldModel;
  const GroundTruthBallModel& theGroundTruthBallModel;
  const ObstacleModel& theObstacleModel;
  const USObstacleGrid& theUSObstacleGrid;
  const RobotPose& theRobotPose;
  const RobotPoseInfo& theRobotPoseInfo;
  const PotentialRobotPose& thePotentialRobotPose;
  const GroundTruthRobotPose& theGroundTruthRobotPose;
  const RobotPoseHypotheses& theRobotPoseHypotheses;
  const RobotsModel& theRobotsModel;
  const GroundTruthRobotsModel& theGroundTruthRobotsModel;
  const FreePartOfOpponentGoalModel& theFreePartOfOpponentGoalModel;
  const SSLVisionData& theSSLVisionData;
  const GroundTruthResult& theGroundTruthResult;
  const FieldCoverage& theFieldCoverage;
  const GlobalFieldCoverage& theGlobalFieldCoverage;

  // BehaviorControl
  const BehaviorControlOutput& theBehaviorControlOutput;
  const KickInfo& theKickInfo;
  const BehaviorLEDRequest& theBehaviorLEDRequest;
  const HeadMotionProposal& theHeadMotionProposal;

  // Sensing
  const FilteredJointData& theFilteredJointData;
  const FilteredJointDataPrev& theFilteredJointDataPrev;
  const FilteredSensorData& theFilteredSensorData;
  const InertiaSensorData& theInertiaSensorData;
  const InspectedInertiaSensorData& theInspectedInertiaSensorData;
  const OrientationData& theOrientationData;
  const GroundTruthOrientationData& theGroundTruthOrientationData;
  const TorsoMatrix& theTorsoMatrix;
  const RobotModel& theRobotModel;

  // MotionControl
  const OdometryData& theOdometryData;
  const GroundTruthOdometryData& theGroundTruthOdometryData;
  const MotionRequest& theMotionRequest;
  const HeadAngleRequest& theHeadAngleRequest;
  const HeadMotionRequest& theHeadMotionRequest;
  const HeadJointRequest& theHeadJointRequest;
  const MotionSelection& theMotionSelection;
  const SpecialActionsOutput& theSpecialActionsOutput;
  const WalkingEngineOutput& theWalkingEngineOutput;
  const WalkingEngineStandOutput& theWalkingEngineStandOutput;
  const BikeEngineOutput& theBikeEngineOutput;
  const MotionInfo& theMotionInfo;
  const UnstableJointRequest& theUnstableJointRequest;

  // Debugging
  const XabslInfo& theXabslInfo;

  PROCESS_WIDE_STORAGE_STATIC(Blackboard) theInstance; /**< The only real instance in the current process. */

  /**
  * The method is a dummy that is called to prevent the compiler from certain
  * optimizations in a method generated in Module.h.
  * It is empty, but important, not defined inline.
  */
  static void distract();

private:
  /**
  * Default constructor.
  */
  Blackboard();

public:
  /**
  * Virtual destructor.
  * Required for derivations of this class.
  */
  virtual ~Blackboard() {}

  /**
  * Assignment operator.
  * Note: copies will share all representations.
  * @param other The instance that is cloned.
  */
  void operator=(const Blackboard& other);

  /**
  * The operator allocates a memory block that is zeroed.
  * Therefore, all members of this class are initialized with 0.
  * @attention This operator is only called if this class is instantiated by
  * a separate call to new, i.e. it cannot be created as a part of another class.
  * @param size The size of the block in bytes.
  * @return A pointer to the block.
  */
  static void* operator new(std::size_t);

  /**
  * The operator frees a memory block.
  * @param p The address of the block to free.
  */
  static void operator delete(void* p);

  friend class Process; /**< The class Process can set theInstance. */
  friend class Cognition; /**< The class Cognition can read theInstance. */
  friend class Motion; /**< The class Motion can read theInstance. */
  friend class Framework; /**< The class Framework can set theInstance. */
};
