/**
* @file BH2011BIKESymbols.cpp
*
* Implementation of class BH2011BIKESymbols.
*
* @author Judith Müller
* @author Tobias Kastner
*/

#include "BH2011BIKESymbols.h"
#include "Modules/MotionControl/BIKEParameters.h"
#include "Tools/Debugging/Modify.h"
#include "Tools/Math/Vector2.h"

void BH2011BIKESymbols::registerSymbols(xabsl::Engine& engine)
{
  engine.registerBooleanInputSymbol("motion.kick_forward", this, &BH2011BIKESymbols::getKickForward);
  engine.registerBooleanInputSymbolDecimalParameter("motion.kick_forward", "motion.kick_forward.y", &ballY);
  engine.registerBooleanInputSymbolDecimalParameter("motion.kick_forward", "motion.kick_forward.x", &ballX);
  engine.registerBooleanInputSymbolBooleanParameter("motion.kick_forward", "motion.kick_forward.mirror", &mirror);
  engine.registerBooleanInputSymbolBooleanParameter("motion.kick_forward", "motion.kick_forward.updates", &updates);
  engine.registerBooleanOutputSymbol("bike.update", &updating);
}

void BH2011BIKESymbols::update()
{
}

bool BH2011BIKESymbols::getKickForward()
{
  float x = ballX;
  float y = ballY;

  if(!updates) motionRequest.bikeRequest.mirror = mirror;

  if(y > 0) y = -y;

  x += 20;
  if(x > 130.f) x = 130.f;

  if(y > -70.f) y = -70.f;  //don't hit yourself
  if(y < -120) y = -120;  //don't go to far

  motionRequest.bikeRequest.bMotionType = BikeRequest::kickForward;

  Vector3<> strikeOut, kickTo, motionDirection(0, 0, 0);

  strikeOut = Vector3<>(-90.f, y, -160.f);
  kickTo = Vector3<>(x, y + 10, -160.f);

  if(motionRequest.bikeRequest.dynPoints.size() != 2)
  {
    motionRequest.bikeRequest.dynPoints.resize(2);
  }

  DynPoint dynRFoot3(Phase::rightFootTra, 3, 0, strikeOut, motionDirection,  Vector3<> (0.f, 0.f, 0.f)), //strikeout
           dynRFoot4(Phase::rightFootTra, 4, 0, kickTo, motionDirection,  Vector3<> (0.f, 0.f, 0.f)); //kickto
  //rFoot in Phase3
  motionRequest.bikeRequest.dynPoints[0] = dynRFoot3;
  //rFoot in Phase4
  motionRequest.bikeRequest.dynPoints[1] = dynRFoot4;

  motionRequest.bikeRequest.dynamical = true;
  motionRequest.bikeRequest.ballSpecial = true;

  return true;
}
