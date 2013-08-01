/**
* @file CameraInfo.cpp
* Implementation of class CameraInfo
*/

#include "CameraInfo.h"

CameraInfo::CameraInfo()
{
  resolutionWidth  = cameraResolutionWidth;
  resolutionHeight = cameraResolutionHeight;

  openingAngleWidth = 0.78674f; // 45.08?  openingAngleHeight  = 0.60349f; // 34.58?
  focalLength = 385.54f;
  opticalCenter.x = 160.0f; // unchecked
  opticalCenter.y = 120.0f; // unchecked

  calcAdditionalConstants();
}

void CameraInfo::calcAdditionalConstants()
{
  focalLenPow2 = focalLength * focalLength;
  focalLenPow4 = focalLenPow2 * focalLenPow2;
  focalLengthInv = 1.0f / focalLength;
}
