#pragma once

#include "Tools/Math/Vector2.h"
#include "Representations/BehaviorControl/KickInfo.h"

struct InputRepresentations;

class KickInfoWrapper
{
private:
  const KickInfo* kickInfo;
  Vector2<> ballPos;
public:
  KickInfoWrapper()
    : kickInfo(0)
  {}

  void init(const InputRepresentations& inputReps);
  void update(const InputRepresentations& inputReps);

  Pose2D getKickPose(int k, bool mirror, const Vector2<> target) const;
};

