#pragma once

#include "Tools/Math/Geometry.h"

namespace bs_Geometry
{
  bool isPointInsideRectangle(const Vector2<> &bottomLeft,
                              const Vector2<> &topRight,
                              const Vector2<> &p);
}
