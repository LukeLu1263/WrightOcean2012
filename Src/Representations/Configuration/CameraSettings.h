/**
* @file CameraSettings.h
* Declaration of a class representing the settings of the PDA camera.
* @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
*/

#pragma once

#include <list>
#include "Tools/Streams/Streamable.h"

/**
* @class Properties
* The class represents the properties of the camera.
*/
class CameraSettings : public Streamable
{
public:
  class V4L2Setting
  {
  public:
    int command;
    int value;
    V4L2Setting()
      : command(0), value(0)
    {}
    V4L2Setting(int command, int value)
      : command(command), value(value)
    {}
    bool operator==(const V4L2Setting& o) const { return command == o.command && value == o.value; }
    bool operator!=(const V4L2Setting& o) const { return !(*this == o); }
    V4L2Setting& operator=(const V4L2Setting& o) { command = o.command; value = o.value; return *this; }
  };
private:
  /**
  * The method streams this class.
  * Implements an abstract method of class Streamable.
  * @param in Pointer to a stream to read from.
  * @param out Pointer to a stream to write to.
  */
  void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN();
    STREAM(exposure.value);
    STREAM(exposureCorrection.value);
    STREAM(gain.value);
    STREAM(red.value);
    STREAM(blue.value);
    STREAM(brightness.value);
    STREAM(contrast.value);
    STREAM(saturation.value);
    STREAM(hue.value);
    STREAM(sharpness.value);
    STREAM(green.value);
    STREAM(uvsatResult.value);
    STREAM(edgeEnhancementFactor.value);
    STREAM(denoiseStrength.value);
    STREAM(contrastCenter.value);
    STREAM(autoExposure.value);
    STREAM(autoWhiteBalance.value);
    STREAM(autoGain.value);
    STREAM(autoBlacklevelCompensation.value);
    STREAM(autoSaturationAdjustment.value);
    STREAM(autoContrastCenter.value);
    STREAM_REGISTER_FINISH();
  }
public:
  static const int numSettings = 15;
  V4L2Setting exposure; /**< The exposure time in the range of [0 .. 1023]. */
  V4L2Setting exposureCorrection; /** The exposure correction in the range of [0 .. 255]. TODO: verify range. */
  V4L2Setting gain; /**< The gain level in the range of [0 .. 127]. */
  V4L2Setting red; /**< White balance red ratio in the range of [0 .. 255]. */
  V4L2Setting blue; /**< White balance blue ratio in the range of [0 .. 255]. */
  V4L2Setting brightness; /* The brightness in range of [0 .. 255] */
  V4L2Setting contrast;   /* The contrast in range of [0 .. 127] */
  V4L2Setting saturation; /* The saturation in range of [0 .. 255] */
  V4L2Setting hue; /* The hue in range [-180 .. 180] */
  V4L2Setting sharpness; /* The sharpness in range of [0 .. 31] */
  V4L2Setting green; /* The awb green channel gain in range of [0 .. 255] */
  V4L2Setting uvsatResult; /* The UV sat result. The actual meaning is a bit unclear. Range [0 .. 15] */
  V4L2Setting edgeEnhancementFactor; /* The edge enhancement factor. The actual meaning is a bit unclear. Range [0 .. 31] */
  V4L2Setting denoiseStrength; /* The denoise strength. The actual meaning is a bit unclear. Range [0 .. 255] */
  V4L2Setting contrastCenter; /* The contrast center. The actual meaning is a bit unclear. Range [0 .. 255] */
  static const int numInitSettings = 6;
  V4L2Setting autoExposure; /* 1: Use auto exposure, 0: disable auto exposure. */
  V4L2Setting autoWhiteBalance; /* 1: Use auto white balance, 0: disable auto white balance. */
  V4L2Setting autoGain; /* 1: Use auto gain correction, 0: disable auto gain correction. */
  V4L2Setting autoBlacklevelCompensation; /* 1: Use auto blacklevel compensation, 0: turn it off. */
  V4L2Setting autoSaturationAdjustment; /* 1: Use UV saturation auto adjustment, 0: turn it off. */
  V4L2Setting autoContrastCenter; /* 1: Use auto contrast center, 0: turn it off. */

  /**
  * Default constructor.
  * Initializes everything with invalid values except the settings for auto features.
  * The settings for auto features are initialized so that they disable the
  * features by default.
  */
  CameraSettings();
  CameraSettings(const CameraSettings& o);
  CameraSettings& operator=(const CameraSettings& o);
  bool operator==(const CameraSettings& o) const;
  bool operator!=(const CameraSettings& o) const;
  std::list<V4L2Setting> getChangesAndAssign(const CameraSettings& other);
  std::list<V4L2Setting> getInitSettings() const;
  std::list<V4L2Setting> getSettings() const;
};
