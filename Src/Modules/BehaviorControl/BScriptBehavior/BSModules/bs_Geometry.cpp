#include "../Util/b-script/src/bsNativeModule.h"
#include "bs_Geometry.h"

namespace bs_Geometry
{

bool isPointInsideRectangle(const Vector2<> &bottomLeft,
                            const Vector2<> &topRight,
                            const Vector2<> &p)
{
  return Geometry::isPointInsideRectangle(bottomLeft, topRight, p);
}

}

BS_MODULE(Geometry,
  BS_REQUIRES(Math);
  registerFunction(module, "isPointInsideRectangle", &bs_Geometry::isPointInsideRectangle);
  );

