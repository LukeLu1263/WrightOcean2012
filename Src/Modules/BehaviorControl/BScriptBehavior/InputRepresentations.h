#pragma once

class FrameInfo;
class GameInfo;
class RobotInfo;
class OwnTeamInfo;
class OpponentTeamInfo;
class FieldDimensions;
class RobotPose;
class BallModel;
class BallHypotheses;
class ObstacleModel;
class RobotsModel;
class MotionInfo;
class FallDownState;
class KickInfo;
class KeyStates;
class GoalPercept;
class RobotModel;
class RobotDimensions;
class TorsoMatrix;

struct InputRepresentations
{
  InputRepresentations(const FrameInfo& frameInfo,
                       const GameInfo& gameInfo,
                       const RobotInfo& robotInfo,
                       const OwnTeamInfo& ownTeamInfo,
                       const OpponentTeamInfo& opponentTeamInfo,
                       const FieldDimensions& fieldDimensions,
                       const RobotPose& robotPose,
                       const BallModel& ballModel,
                       const BallHypotheses& ballHypotheses,
                       const ObstacleModel& obstacleModel,
                       const RobotsModel& robotsModel,
                       const MotionInfo& motionInfo,
                       const FallDownState& fallDownState,
                       const KickInfo& kickInfo,
                       const KeyStates& keyStates,
                       const GoalPercept& goalPercept,
                       const RobotModel& robotModel,
                       const RobotDimensions& robotDimensions,
                       const TorsoMatrix& torsoMatrix)
    : theFrameInfo(frameInfo),
      theGameInfo(gameInfo),
      theRobotInfo(robotInfo),
      theOwnTeamInfo(ownTeamInfo),
      theOpponentTeamInfo(opponentTeamInfo),
      theFieldDimensions(fieldDimensions),
      theRobotPose(robotPose),
      theBallModel(ballModel),
      theBallHypotheses(ballHypotheses),
      theObstacleModel(obstacleModel),
      theRobotsModel(robotsModel),
      theMotionInfo(motionInfo),
      theFallDownState(fallDownState),
      theKickInfo(kickInfo),
      theKeyStates(keyStates),
      theGoalPercept(goalPercept),
      theRobotModel(robotModel),
      theRobotDimensions(robotDimensions),
      theTorsoMatrix(torsoMatrix)
  {}

  const FrameInfo& theFrameInfo;
  const GameInfo& theGameInfo;
  const RobotInfo& theRobotInfo;
  const OwnTeamInfo& theOwnTeamInfo;
  const OpponentTeamInfo& theOpponentTeamInfo;
  const FieldDimensions& theFieldDimensions;
  const RobotPose& theRobotPose;
  const BallModel& theBallModel;
  const BallHypotheses& theBallHypotheses;
  const ObstacleModel& theObstacleModel;
  const RobotsModel& theRobotsModel;
  const MotionInfo& theMotionInfo;
  const FallDownState& theFallDownState;
  const KickInfo& theKickInfo;
  const KeyStates& theKeyStates;
  const GoalPercept& theGoalPercept;
  const RobotModel& theRobotModel;
  const RobotDimensions& theRobotDimensions;
  const TorsoMatrix& theTorsoMatrix;
};

