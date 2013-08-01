#pragma once

#include "Representations/Perception/GoalPercept.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "../InputRepresentations.h"

class GoalWrapper
{
public:
  struct GoalInfo
  {
    int completeLastSeen,
        anyPostLastSeen;
  };

  GoalInfo own,
           opp;

  void init(const InputRepresentations& inputReps)
  {}

  void update(const InputRepresentations& inputReps)
  {
    const GoalPercept& theGoalPercept = inputReps.theGoalPercept;
    const FrameInfo& theFrameInfo = inputReps.theFrameInfo;

    if(theGoalPercept.posts[GoalPercept::LEFT_OWN].timeWhenLastSeen == theFrameInfo.time &&
       theGoalPercept.posts[GoalPercept::RIGHT_OWN].timeWhenLastSeen == theFrameInfo.time)
      own.completeLastSeen = theFrameInfo.time;
    if(theGoalPercept.posts[GoalPercept::LEFT_OWN].timeWhenLastSeen == theFrameInfo.time ||
       theGoalPercept.posts[GoalPercept::RIGHT_OWN].timeWhenLastSeen == theFrameInfo.time ||
       theGoalPercept.posts[GoalPercept::UNKNOWN_OWN].timeWhenLastSeen == theFrameInfo.time)
      own.anyPostLastSeen = theFrameInfo.time;
    if(theGoalPercept.posts[GoalPercept::LEFT_OPPONENT].timeWhenLastSeen == theFrameInfo.time &&
       theGoalPercept.posts[GoalPercept::RIGHT_OPPONENT].timeWhenLastSeen == theFrameInfo.time)
      own.completeLastSeen = theFrameInfo.time;
    if(theGoalPercept.posts[GoalPercept::LEFT_OPPONENT].timeWhenLastSeen == theFrameInfo.time ||
       theGoalPercept.posts[GoalPercept::RIGHT_OPPONENT].timeWhenLastSeen == theFrameInfo.time ||
       theGoalPercept.posts[GoalPercept::UNKNOWN_OPPONENT].timeWhenLastSeen == theFrameInfo.time)
      own.anyPostLastSeen = theFrameInfo.time;
  }
};

