#include "../Util/b-script/src/bsNativeModule.h"
#include "bs_Math.h"

BS_MODULE(Math,
   //Vector2<>
  registerClass<Vector2<> >(module, "Vector2f", "Vector2<>")
    .var("x", &Vector2<>::x)
    .var("y", &Vector2<>::y)
    .func("abs", &Vector2<>::abs)
    .func("squareAbs", &Vector2<>::squareAbs)
    .func("angle", &Vector2<>::angle)
    .func("_plus_", &Vector2<>::operator+, "operator+")
    .func("_min_", (Vector2<> (Vector2<>::*)(const Vector2<>&) const)&Vector2<>::operator-, "operator-")
    .func("_mul_", (Vector2<> (Vector2<>::*)(const float &) const)&Vector2<>::operator*, "operator*")
    .func("_div_", &Vector2<>::operator/, "operator/")
    ;

  //Vector3<>
  registerClass<Vector3<> >(module, "Vector3f", "Vector3<>")
    .var("x", &Vector3<>::x)
    .var("y", &Vector3<>::y)
    .var("z", &Vector3<>::z)
    .func("abs", &Vector3<>::abs)
    .func("squareAbs", &Vector3<>::squareAbs)
    .func("_plus_", &Vector3<>::operator+)
    .func("_min_", (Vector3<> (Vector3<>::*)(const Vector3<>&) const)&Vector3<>::operator-)
    .func("_mul_", (Vector3<> (Vector3<>::*)(const float &) const)&Vector3<>::operator*)
    .func("_div_", &Vector3<>::operator/)
    ;

  //Pose2D
  registerClass<Pose2D>(module, "Pose2D", "Pose2D")
    .var("translation", &Pose2D::translation)
    .var("rotation", &Pose2D::rotation)
    .func("invert", &Pose2D::invert)
    .func("_mul_", &Pose2D::operator*, "operator*")
    ;

  //all kinds of functions (for now ;-))
  registerFunction(module, "fromDegrees", &fromDegrees<float>, "fromDegrees");
  registerFunction(module, "normalize", &normalize<float>, "normalize");
  registerFunction(module, "fabs", (float(*)(float))&fabs, "fabs");
  registerFunction(module, "random", &randomFloat, "randomFloat");
  registerFunction(module, "sqrf", &sqr<float>, "sqr");
  registerFunction(module, "sqri", &sqr<int>, "sqr");
  registerFunction(module, "sgnf", &sgn<float>, "sgn");
  registerFunction(module, "sgni", &sgn<int>, "sgn");

  registerConstValue(module, "pi", (float)pi, "(float)pi");
  );

