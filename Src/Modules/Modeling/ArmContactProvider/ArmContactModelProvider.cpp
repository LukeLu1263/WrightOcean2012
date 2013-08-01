/**
* @file ArmContactModelProvider.h
*
* Implementation of class ArmContactModelProvider.
* @author <a href="mailto:fynn@informatik.uni-bremen.de">Fynn Feldpausch</a>
*/


#include "Tools/Debugging/DebugDrawings.h"
#include "ArmContactModelProvider.h"
#include "Platform/SoundPlayer.h"


MAKE_MODULE(ArmContactModelProvider, Modeling);


ArmContactModelProvider::ArmContactModelProvider():
  errorLeftX(0), errorLeftY(0), errorRightX(0), errorRightY(0), frameDelay(2), soundDelay(1000), lastSoundTime(0)
{
  p.errorXThreshold = 7.5;
  p.errorYThreshold = 3.75;
  p.errorXDecrease = 4.0;
  p.errorYDecrease = 3.0;
  p.debugMode = false;
}

bool ArmContactModelProvider::checkArm(bool left)
{
  /* Calculate arm diffs */
  struct ArmAngles angles = angleBuffer[frameDelay];
  float diffX = fabs(toDegrees(left
                               ? angles.leftX - theFilteredJointData.angles[JointData::LShoulderPitch]
                               : angles.rightX - theFilteredJointData.angles[JointData::RShoulderPitch]));
  float diffY = fabs(toDegrees(left
                               ? angles.leftY - theFilteredJointData.angles[JointData::LShoulderRoll]
                               : angles.rightY - theFilteredJointData.angles[JointData::RShoulderRoll]));

  if(left)
  {
    errorLeftX = min(errorLeftX + diffX, 90.0f);
    errorLeftY = min(errorLeftY + diffY, 90.0f);
  }
  else
  {
    errorRightX = min(errorRightX + diffX, 90.0f);
    errorRightY = min(errorRightY + diffY, 90.0f);
  }

  /* Detect collisions */
  if((left && (errorLeftX > p.errorXThreshold || errorLeftY > p.errorYThreshold)) ||
     (!left && (errorRightX > p.errorXThreshold || errorRightY > p.errorYThreshold)))
    return true;

  return false;
}

void ArmContactModelProvider::update(ArmContactModel& ArmContactModel)
{
  MODIFY("module:ArmContactModelProvider:parameters", p);

  DECLARE_PLOT("module:ArmContactModelProvider:contactLeft");
  DECLARE_PLOT("module:ArmContactModelProvider:contactRight");
  PLOT("module:ArmContactModelProvider:contactLeft", ArmContactModel.contactLeft ? 7.5 : 2.5);
  PLOT("module:ArmContactModelProvider:contactRight", ArmContactModel.contactRight ? 7.5 : 2.5);

  DECLARE_PLOT("module:ArmContactModelProvider:errorLeftX");
  DECLARE_PLOT("module:ArmContactModelProvider:errorRightX");
  DECLARE_PLOT("module:ArmContactModelProvider:errorLeftY");
  DECLARE_PLOT("module:ArmContactModelProvider:errorRightY");
  PLOT("module:ArmContactModelProvider:errorLeftX", errorLeftX);
  PLOT("module:ArmContactModelProvider:errorRightX", errorRightX);
  PLOT("module:ArmContactModelProvider:errorLeftY", errorLeftY);
  PLOT("module:ArmContactModelProvider:errorRightY", errorRightY);

  /* Buffer arm angles */
  struct ArmAngles angles;
  angles.leftX = theJointRequest.angles[JointData::LShoulderPitch];
  angles.leftY = theJointRequest.angles[JointData::LShoulderRoll];
  angles.rightX = theJointRequest.angles[JointData::RShoulderPitch];
  angles.rightY = theJointRequest.angles[JointData::RShoulderRoll];
  angleBuffer.add(angles);

  /* Decrease errors */
  errorLeftX = max(0.0f, errorLeftX - p.errorXDecrease);
  errorLeftY = max(0.0f, errorLeftY - p.errorYDecrease);
  errorRightX = max(0.0f, errorRightX - p.errorXDecrease);
  errorRightY = max(0.0f, errorRightY - p.errorYDecrease);

  /* Reset in case of a fall */
  if(theFallDownState.state == FallDownState::onGround)
  {
    errorLeftX = 0.0f;
    errorLeftY = 0.0f;
    errorRightX = 0.0f;
    errorRightY = 0.0f;
  }

  /* Check for arm contact */
  // motion types to take into account: stand, walk (if the robot is upright)
  if((theMotionInfo.motion == MotionInfo::stand || theMotionInfo.motion == MotionInfo::walk) &&
     (theFallDownState.state == FallDownState::upright || theFallDownState.state == FallDownState::staggering))
  {
    bool left = checkArm(true);
    bool right = checkArm(false);
    if(p.debugMode && theFrameInfo.getTimeSince(lastSoundTime) > soundDelay &&
       ((left && !ArmContactModel.contactLeft) || (right && !ArmContactModel.contactRight)))
    {
      lastSoundTime = theFrameInfo.time;
      SoundPlayer::play("arm.wav");
    }
    ArmContactModel.contactLeft = left;
    ArmContactModel.contactRight = right;
  }
  else
  {
    ArmContactModel.contactLeft = false;
    ArmContactModel.contactRight = false;
  }
}
