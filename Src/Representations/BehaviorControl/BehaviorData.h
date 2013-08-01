/**
* @file Representations/BehaviorControl/BehaviorData.h
* The file declares a class that containts data about the current behavior state.
* @author Colin Graf
*/

#pragma once

#include "Tools/Math/Vector2.h"
#include "Tools/Xabsl/B-Human/BHXabslMessage.h"
#include "Tools/Enum.h"

/**
* @class BehaviorData
* A class that containts data about the current behavior state.
*/
class BehaviorData : public Streamable
{
public:
  ENUM(Role,
    undefined,
    firstRole,
    keeper = firstRole,
    supporter,
    striker,
    defender,
    offensiveSupporter,
    defensiveSupporter
  );
  Role role; /**< A dynamically chosen role. */

  ENUM(Action,
    unknown,
    dribble,
    goToBall,
    searchForBall,
    goToTarget,
    prepareKick,
    kick,
    kickSidewards,
    pass,
    block,
    hold,
    standUp,
    patrol,
    passBeforeGoal,
    kickoff,
    goAround,
    waitForPass,
    preparePass,
    parry,
    guard,
    defending,
    receive,
    duel
  );
  Action action; /**< What is the robot doing in general? */

  ENUM(TeamColor,
    red,
    blue
  );

  float estimatedTimeToReachBall;   /**< The estimated time to reach the kick pose (in ms) */
  bool kickoffInProgress;           /**< Whether the kickoff is currently in progress or not. */
  char selectedKickoff;              /**< The kickoff strategy that has been selected */
  char kickoffPositionAssigment[3]; /**< Assignment of robot numbers to positions (order as specified in current kickoff strategy definition)*/
  TeamColor teamColor;

  /**
  * Default constructor.
  */
  BehaviorData() :
    role(undefined),
    action(unknown),
    estimatedTimeToReachBall(10000000.0f),
    kickoffInProgress(false),
    selectedKickoff(-1),
    teamColor(red)
  {}

  BHXabslMessage xabslMessage;  /**< Xabsl message transmitted from or to other cooperating agents */

private:
  /**
  * The method makes the object streamable.
  * @param in The stream from which the object is read (if in != 0).
  * @param out The stream to which the object is written (if out != 0).
  */
  virtual void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN();
    STREAM_AS_UCHAR(role);
    STREAM_AS_UCHAR(action);
    STREAM(estimatedTimeToReachBall);
    STREAM(kickoffInProgress);
    STREAM(selectedKickoff);
    STREAM(kickoffPositionAssigment);
    BHXabslMessageCompressed message(xabslMessage);
    STREAM(message);
    xabslMessage = message.unpack();
    STREAM_AS_UCHAR(teamColor);
    STREAM_REGISTER_FINISH();
  }
};
