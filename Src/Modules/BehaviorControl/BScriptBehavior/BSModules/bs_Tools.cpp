#include "../Util/b-script/src/bsNativeModule.h"
#include "bs_Tools.h"

namespace bs_Tools
{
void Trajectory::addPoint(float x, float y, int duration)
{
  points.push_back(TrajPoint(Vector2<>(x, y), duration));
  summedDuration += duration;
}

Vector2<> Trajectory::getAt(int duration) const
{
  int d = 0;
  size_t i;
  for(i = 0; i < points.size(); ++i)
  {
    d += points.at(i).duration;
    if(d >= duration)
      break;
  }

  if(i == 0)
    return points.at(0).p;
  if(d <= duration || i == points.size())
    return points.at(points.size() - 1).p;

  const TrajPoint &p1 = points.at(i-1),
                  &p2 = points.at(i);
  int relativeDuration = duration - (d - p2.duration);
  float ratio = relativeDuration / float(p2.duration);
  return p1.p * (1 - ratio) + p2.p * ratio;
}
}

BS_MODULE(Tools,
  BS_REQUIRES(Math);

  registerClass<bs_Tools::Trajectory>(module, "Trajectory", "bs_Tools::Trajectory")
    .funcVoid("addPoint", &bs_Tools::Trajectory::addPoint)
    .func("getDuration", &bs_Tools::Trajectory::getDuration)
    .func("getAt", &bs_Tools::Trajectory::getAt)
    ;

  );

