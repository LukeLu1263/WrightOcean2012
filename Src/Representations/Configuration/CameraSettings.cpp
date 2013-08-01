/**
* @file CameraSettings.cpp
* Implementation of class CameraSettings.
*
* @author Felix Wenk
*/

#include "CameraSettings.h"
#include "Platform/Camera.h"

#ifdef CAMERA_INCLUDED
#undef __STRICT_ANSI__
#include <linux/videodev2.h>
#define __STRICT_ANSI__

#ifndef V4L2_CID_AUTOEXPOSURE
#  define V4L2_CID_AUTOEXPOSURE     (V4L2_CID_BASE+32)
#endif

#ifndef V4L2_CID_SAT_AUTO
#  define V4L2_CID_SAT_AUTO  (V4L2_CID_BASE+36)
#endif

#ifndef V4L2_CID_EXPOSURE_CORRECTION
#  define V4L2_CID_EXPOSURE_CORRECTION  (V4L2_CID_BASE+34)
#endif

#ifndef V4L2_CID_BACKLIGHT_COMPENSATION
#  define V4L2_CID_BACKLIGHT_COMPENSATION  (V4L2_CID_BASE+28)
#endif

#ifndef V4L2_CID_SHARPNESS
#  define V4L2_CID_SHARPNESS  (V4L2_CID_BASE+27)
#endif

#ifndef V4L2_CID_AWB_G_CHANNEL_GAIN
#  define V4L2_CID_AWB_G_CHANNEL_GAIN (V4L2_CID_BASE+38)
#endif

#ifndef V4L2_CID_UVSAT_RESULT
#  define V4L2_CID_UVSAT_RESULT (V4L2_CID_BASE+39)
#endif

#ifndef V4L2_CID_EDGE_ENH_FACTOR
#  define V4L2_CID_EDGE_ENH_FACTOR (V4L2_CID_BASE+40)
#endif

#ifndef V4L2_CID_DENOISE_STRENGTH
#  define V4L2_CID_DENOISE_STRENGTH (V4L2_CID_BASE+41)
#endif

#ifndef V4L2_CID_AUTO_CONTRAST_CENTER
#  define V4L2_CID_AUTO_CONTRAST_CENTER (V4L2_CID_BASE+42)
#endif

#ifndef V4L2_CID_CONTRAST_CENTER
#  define V4L2_CID_CONTRAST_CENTER (V4L2_CID_BASE+43)
#endif

CameraSettings::CameraSettings()
  : exposure(V4L2Setting(V4L2_CID_EXPOSURE, -1)),
    exposureCorrection(V4L2Setting(V4L2_CID_EXPOSURE_CORRECTION, -1)),
    gain(V4L2Setting(V4L2_CID_GAIN, -1)),
    red(V4L2Setting(V4L2_CID_RED_BALANCE, -1)),
    blue(V4L2Setting(V4L2_CID_BLUE_BALANCE, -1)),
    brightness(V4L2Setting(V4L2_CID_BRIGHTNESS, -1)),
    contrast(V4L2Setting(V4L2_CID_CONTRAST, -1)),
    saturation(V4L2Setting(V4L2_CID_SATURATION, 1)),
    hue(V4L2Setting(V4L2_CID_HUE, 0)),
    sharpness(V4L2Setting(V4L2_CID_SHARPNESS, -1)),
    green(V4L2Setting(V4L2_CID_AWB_G_CHANNEL_GAIN, -1)),
    uvsatResult(V4L2Setting(V4L2_CID_UVSAT_RESULT, -1)),
    edgeEnhancementFactor(V4L2Setting(V4L2_CID_EDGE_ENH_FACTOR, -1)),
    denoiseStrength(V4L2Setting(V4L2_CID_DENOISE_STRENGTH, -1)),
    contrastCenter(V4L2Setting(V4L2_CID_CONTRAST_CENTER, -1)),
    autoExposure(V4L2Setting(V4L2_CID_AUTOEXPOSURE, 0)),
    autoWhiteBalance(V4L2Setting(V4L2_CID_AUTO_WHITE_BALANCE, 0)),
    autoGain(V4L2Setting(V4L2_CID_AUTOGAIN, 0)),
    autoBlacklevelCompensation(V4L2Setting(V4L2_CID_BACKLIGHT_COMPENSATION, 1)),
    autoSaturationAdjustment(V4L2Setting(V4L2_CID_SAT_AUTO, 0)),
    autoContrastCenter(V4L2Setting(V4L2_CID_AUTO_CONTRAST_CENTER, 0))
{}
#else // !CAMERA_INCLUDED
CameraSettings::CameraSettings() {}
#endif

CameraSettings::CameraSettings(const CameraSettings& o)
{
  *this = o;
}

bool CameraSettings::operator==(const CameraSettings& o) const
{
  const V4L2Setting* thisSettings = &exposure;
  const V4L2Setting* otherSettings = &o.exposure;
  for(int i = 0; i < numSettings; i++)
    if(thisSettings[i] != otherSettings[i])
      return false;

  thisSettings = &autoExposure;
  otherSettings = &o.autoExposure;
  for(int i = 0; i < numInitSettings; i++)
    if(thisSettings[i] != otherSettings[i])
      return false;

  return true;
}

bool CameraSettings::operator!=(const CameraSettings& o) const
{
  return !(*this == o);
}

std::list<CameraSettings::V4L2Setting> CameraSettings::getChangesAndAssign(const CameraSettings& other)
{
  std::list<V4L2Setting> changes;
  V4L2Setting* thisSettings = &exposure;
  const V4L2Setting* otherSettings = &other.exposure;
  for(int i = 0; i < numSettings; i++)
  {
    if(thisSettings[i] != otherSettings[i])
    {
      thisSettings[i] = otherSettings[i];
      changes.push_back(thisSettings[i]);
    }
  }
  thisSettings = &autoExposure;
  otherSettings = &other.autoExposure;
  for(int i = 0; i < numInitSettings; i++)
  {
    if(thisSettings[i] != otherSettings[i])
    {
      thisSettings[i] = otherSettings[i];
      changes.push_back(thisSettings[i]);
    }
  }
  return changes;
}

std::list<CameraSettings::V4L2Setting> CameraSettings::getInitSettings() const
{
  const V4L2Setting* start = &autoExposure;
  std::list<V4L2Setting> settings;
  for(int i = 0; i < numInitSettings; i++)
    settings.push_back(start[i]);
  return settings;
}

std::list<CameraSettings::V4L2Setting> CameraSettings::getSettings() const
{
  std::list<V4L2Setting> settings = getInitSettings();
  const V4L2Setting* start = &exposure;
  for(int i = 0; i < numSettings; i++)
    settings.push_back(start[i]);
  return settings;
}

CameraSettings& CameraSettings::operator=(const CameraSettings& o)
{
  V4L2Setting* settings = &exposure;
  const V4L2Setting* otherSettings = &o.exposure;
  for(int i = 0; i < numSettings; i++)
    settings[i] = otherSettings[i];

  settings = &autoExposure;
  otherSettings = &o.autoExposure;
  for(int i = 0; i < numInitSettings; i++)
    settings[i] = otherSettings[i];

  return *this;
}
