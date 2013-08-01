/**
* \file Platform/linux/NaoCamera.h
* Interface to the Nao camera using linux-uvc.
* \author Colin Graf
*/

#pragma once

#include "Tools/Streams/InStreams.h"
#include "Representations/Configuration/CameraSettings.h"
#include "Representations/Perception/ImageInfo.h"

/**
* \class NaoCamera
* Interface class to the Nao camera.
*/
class NaoCamera
{
public:

  /** Constructor. */
  NaoCamera();

  /** Destructor. */
  ~NaoCamera();

  /** Verify Nao version. Returns true if the Nao version is at least 3. */
  bool verifyNaoVersion();

  /**
  * Sets the camera control settings to the camera.
  * \param settings The settings.
  */
  void setSettings(const CameraSettings& settings);

  /**
  * Requests the current camera control settings of the camera.
  * \return The settings.
  */
  const CameraSettings& getSettings() const { return settings; };

  /**
  * The method blocks till a new image arrives.
  * \return true (except a not manageable exception occurs)
  */
  bool captureNew();

  /**
  * The last captured image.
  * \return The image data buffer.
  */
  const unsigned char* getImage() const;

  /**
  * Timestamp of the last captured image.
  * \return The timestamp.
  */
  unsigned getTimeStamp() const;

  ImageInfo::Camera switchToUpper();
  ImageInfo::Camera switchToLower();
  /**
   * Switches the current camera.
   *
   * camera: The camera to switch to.
   */
  ImageInfo::Camera switchCamera(ImageInfo::Camera camera);

  ImageInfo::Camera getCurrentCamera() { return currentCamera; }

  /**
   * Asserts that the actual camera settings are correct.
   */
  void assertCameraSettings();

  /**
   * Unconditional write of the camera settings
   */
  void writeCameraSettings();
private:
  static NaoCamera* theInstance; /**< The only instance of this class. */

  CameraSettings settings; /**< The camera control settings. */

  ImageInfo::Camera currentCamera; /**< The current camera in use */

  enum
  {
    frameBufferCount = 3, /**< Amount of available frame buffers. */
    WIDTH = 640,
    HEIGHT = 480,
    SIZE = WIDTH * HEIGHT * 2
  };
  int fd; /**< The file descriptor for the video device. */
  void* mem[frameBufferCount]; /**< Frame buffer addresses. */
  int memLength[frameBufferCount]; /**< The length of each frame buffer. */
  struct v4l2_buffer* buf; /**< Reusable parameter struct for some ioctl calls. */
  struct v4l2_buffer* currentBuf; /**< The last dequeued frame buffer. */
  unsigned timeStamp; /**< Timestamp of the last captured image. */

  /**
  * Requests the value of a camera control setting from camera.
  * \param id The setting id.
  * \return The value.
  */
  int getControlSetting(unsigned int id);

  /**
  * Sets the value of a camera control setting to camera.
  * \param id The setting id.
  * \param value The value.
  * \return True on success.
  */
  bool setControlSetting(unsigned int id, int value);

  /**
   * Applies a collection of camera control settings.
   *
   * \param list of control settings.
   * \return True if every control setting has been applied, false otherwise.
   */
  bool setControlSettings(std::list<CameraSettings::V4L2Setting> controlsettings);

  /** Open and connects to the i2c adapter device. */
  int openI2CAdapter();

  /** Closes the I2C adapter referenced by the file descriptor 'filedes' */
  void closeI2CAdapter(int filedes);

  void initSelectCamera(ImageInfo::Camera camera);
  void initOpenVideoDevice();
  void initSetCameraDefaults();
  void initSetImageFormat();
  void initSetFrameRate();
  void initRequestAndMapBuffers();
  void initQueueAllBuffers();
  void initDefaultControlSettings();
  void initResetCrop();
  void startCapturing();
};
