#include "Representations/Modeling/BallModel.h"
#include "../InputRepresentations.h"

#include "Tools/Debugging/Asserts.h"
#include "KickInfoWrapper.h"

void KickInfoWrapper::init(const InputRepresentations& inputReps)
{
  kickInfo = &inputReps.theKickInfo;
}

void KickInfoWrapper::update(const InputRepresentations& inputReps)
{
  ASSERT(kickInfo);
  ballPos = inputReps.theRobotPose * inputReps.theBallModel.estimate.position;
}

Pose2D KickInfoWrapper::getKickPose(int k, bool mirror, const Vector2<> target) const
{
  ASSERT(kickInfo);
  Pose2D ballPose((target - ballPos).angle(), ballPos);
  const KickInfo::Kick& kick = kickInfo->kicks[k + (mirror ? 1 : 0)];
  ballPose.rotate(kick.rotationOffset);
  ballPose.translate(kick.ballOffset);
  //Pose2D offset(kick.rotationOffset, kick.ballOffset);
  //ballPose += offset;
  return ballPose;
}

