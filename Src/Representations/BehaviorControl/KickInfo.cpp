#include "KickInfo.h"

KickInfo::KickInfo()
{
  kicks[bikeForward] = KickInfo::Kick(0, Vector2<>(-190, 70), 999999.f, 1000, MotionRequest::bike, BikeRequest::kickForward, false); // dribble position 1
  kicks[bikeForward + 1] = KickInfo::Kick(0, Vector2<>(-190, -70), 999999.f, 1000, MotionRequest::bike, BikeRequest::kickForward, true); // dribble position 2

#ifdef TARGET_SIM
  kicks[bikeForward].ballOffset.x = -150;
  kicks[bikeForward + 1].ballOffset.x = -150;
#endif
}

