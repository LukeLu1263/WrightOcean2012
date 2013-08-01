#pragma once

#include <vector>
#include "Tools/Math/Vector2.h"

namespace bs_Tools
{
  class Trajectory
  {
  public:
    Trajectory()
      : summedDuration(0)
    {}

    void addPoint(float x, float y, int duration);
    Vector2<> getAt(int duration) const;
    int getDuration() const { return summedDuration; }

  private:
    class TrajPoint
    {
    public:
      TrajPoint(const Vector2<> p, int duration)
        : p(p),
          duration(duration)
      {}

      Vector2<> p;
      int duration;
    };

    std::vector<TrajPoint> points;
    int summedDuration;
  };
}
