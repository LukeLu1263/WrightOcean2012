#include "Tools/Debugging/Asserts.h"
#include "Tools/Math/Pose3D.h"
#include "Tools/Math/Geometry.h"
#include "Representations/Sensing/TorsoMatrix.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "BSBikeDynPoints.h"

void BSBikeDynPoints::set(BikeRequest::BMotionID bmotion, const Vector2<> &ball, bool mirror)
{
  Vector2<> specialBall = calculateBallInRobotOrigin(ball);
  motionRequest->motion = MotionRequest::bike;
  if(bmotion == BikeRequest::kickForward)
  {
    float x = specialBall.x;
    float y = specialBall.y;

    motionRequest->bikeRequest.mirror = (y > 0.f);
    if(motionRequest->bikeRequest.mirror == true) y = -y;

    x += 20;
    if(x > 130.f) x = 130.f;

    if(y > -40.f) y = -40.f;  //don't hit yourself
    if(y < -120) y = -120;  //don't go to far

    motionRequest->bikeRequest.bMotionType = BikeRequest::kickForward;

    Vector3<> strikeOut, kickTo, motionDirection(0, 0, 0);

    strikeOut = Vector3<>(-90.f, y, -160.f);
    kickTo = Vector3<>(x, y + 10, -160.f);

    if(motionRequest->bikeRequest.dynPoints.size() != 2)
    {
      motionRequest->bikeRequest.dynPoints.resize(2);
    }


    DynPoint dynRFoot3(Phase::rightFootTra, 3, 0, strikeOut, motionDirection,  Vector3<> (0.f, 0.f, 0.f)), //strikeout
             dynRFoot4(Phase::rightFootTra, 4, 0, kickTo, motionDirection,  Vector3<> (0.f, 0.f, 0.f)); //kickto
    //rFoot in Phase3
    motionRequest->bikeRequest.dynPoints[0] = dynRFoot3;
    //rFoot in Phase4
    motionRequest->bikeRequest.dynPoints[1] = dynRFoot4;

    motionRequest->bikeRequest.dynamical = true;
    motionRequest->bikeRequest.ballSpecial = true;

    if(abs(specialBall.y) < 65 || abs(specialBall.y) > 80)
    {
      //hmmmpf
    }
  }
  else
    ASSERT(false);
}

Vector2<> BSBikeDynPoints::calculateBallInRobotOrigin(const Vector2<>& ballRel)
{

  // calculate "center of hip" position from left foot
  Pose3D fromLeftFoot(torsoMatrix.rotation);
  fromLeftFoot.conc(robotModel.limbs[MassCalibration::footLeft]);
  fromLeftFoot.translate(0, 0, -robotDimensions.heightLeg5Joint);
  //fromLeftFoot.translation *= -1.;
  fromLeftFoot.rotation = torsoMatrix.rotation;

  // calculate "center of hip" position from right foot
  Pose3D fromRightFoot(torsoMatrix.rotation);
  fromRightFoot.conc(robotModel.limbs[MassCalibration::footRight]);
  fromRightFoot.translate(0, 0, -robotDimensions.heightLeg5Joint);
// fromRightFoot.translation *= -1.;
  fromRightFoot.rotation = torsoMatrix.rotation;

  // determine used foot
  const bool useLeft = fromLeftFoot.translation.z < fromRightFoot.translation.z;

  // calculate foot span
  const Vector3<> newFootSpan(fromRightFoot.translation - fromLeftFoot.translation);

  // and construct the matrix
  Pose3D newTorsoMatrix;
  newTorsoMatrix.translate(newFootSpan.x / (useLeft ? 2.f : -2.f), newFootSpan.y / (useLeft ? 2.f : -2.f), 0);
  //newTorsoMatrix.conc(useLeft ? fromLeftFoot : fromRightFoot);

  const Vector3<> foot(useLeft ? fromLeftFoot.translation : fromRightFoot.translation);


  return Vector2<>(ballRel.x + newTorsoMatrix.translation.x + foot.x, ballRel.y + newTorsoMatrix.translation.y + foot.y);
}

