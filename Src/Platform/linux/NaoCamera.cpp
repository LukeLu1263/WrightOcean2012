/**
* \file Platform/linux/NaoCamera.cpp
* Interface to the Nao camera using linux-uvc.
* \author Colin Graf
* \author Thomas Röfer
*/

#if defined(TARGET_SIM) && !defined(NO_NAO_EXTENSIONS)
#define NO_NAO_EXTENSIONS
#endif

#include <cstring>
#include <cstdio>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <errno.h>
#include <poll.h>
#ifdef USE_USERPTR
#include <malloc.h> // memalign
#endif

#include "NaoCamera.h"
#include "Platform/BHAssert.h"
#include "Platform/SystemCall.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Tools/Debugging/Debugging.h"
#include "Tools/Streams/InStreams.h"

#undef __STRICT_ANSI__
#include <linux/videodev2.h>
#include <linux/version.h>
#include <linux/i2c-dev.h>
#define __STRICT_ANSI__

#ifndef V4L2_CID_AUTOEXPOSURE
#  define V4L2_CID_AUTOEXPOSURE     (V4L2_CID_BASE+32)
#endif

#ifndef V4L2_CID_CAM_INIT
#  define V4L2_CID_CAM_INIT         (V4L2_CID_BASE+33)
#endif

#ifndef V4L2_CID_AUDIO_MUTE
#  define V4L2_CID_AUDIO_MUTE       (V4L2_CID_BASE+9)
#endif

#ifndef V4L2_CID_POWER_LINE_FREQUENCY
#  define V4L2_CID_POWER_LINE_FREQUENCY  (V4L2_CID_BASE+24)
enum v4l2_power_line_frequency
{
  V4L2_CID_POWER_LINE_FREQUENCY_DISABLED  = 0,
  V4L2_CID_POWER_LINE_FREQUENCY_50HZ  = 1,
  V4L2_CID_POWER_LINE_FREQUENCY_60HZ  = 2,
};

#define V4L2_CID_HUE_AUTO      (V4L2_CID_BASE+25)
#define V4L2_CID_WHITE_BALANCE_TEMPERATURE  (V4L2_CID_BASE+26)
#define V4L2_CID_SHARPNESS      (V4L2_CID_BASE+27)
#define V4L2_CID_BACKLIGHT_COMPENSATION   (V4L2_CID_BASE+28)

#define V4L2_CID_CAMERA_CLASS_BASE     (V4L2_CTRL_CLASS_CAMERA | 0x900)
#define V4L2_CID_CAMERA_CLASS       (V4L2_CTRL_CLASS_CAMERA | 1)

#define V4L2_CID_EXPOSURE_AUTO      (V4L2_CID_CAMERA_CLASS_BASE+1)
enum  v4l2_exposure_auto_type
{
  V4L2_EXPOSURE_MANUAL = 0,
  V4L2_EXPOSURE_AUTO = 1,
  V4L2_EXPOSURE_SHUTTER_PRIORITY = 2,
  V4L2_EXPOSURE_APERTURE_PRIORITY = 3
};
#define V4L2_CID_EXPOSURE_ABSOLUTE    (V4L2_CID_CAMERA_CLASS_BASE+2)
#define V4L2_CID_EXPOSURE_AUTO_PRIORITY    (V4L2_CID_CAMERA_CLASS_BASE+3)

#define V4L2_CID_PAN_RELATIVE      (V4L2_CID_CAMERA_CLASS_BASE+4)
#define V4L2_CID_TILT_RELATIVE      (V4L2_CID_CAMERA_CLASS_BASE+5)
#define V4L2_CID_PAN_RESET      (V4L2_CID_CAMERA_CLASS_BASE+6)
#define V4L2_CID_TILT_RESET      (V4L2_CID_CAMERA_CLASS_BASE+7)

#define V4L2_CID_PAN_ABSOLUTE      (V4L2_CID_CAMERA_CLASS_BASE+8)
#define V4L2_CID_TILT_ABSOLUTE      (V4L2_CID_CAMERA_CLASS_BASE+9)

#define V4L2_CID_FOCUS_ABSOLUTE      (V4L2_CID_CAMERA_CLASS_BASE+10)
#define V4L2_CID_FOCUS_RELATIVE      (V4L2_CID_CAMERA_CLASS_BASE+11)
#define V4L2_CID_FOCUS_AUTO      (V4L2_CID_CAMERA_CLASS_BASE+12)

#endif /* LINUX_VERSION_CODE < KERNEL_VERSION(2,6,26) */

NaoCamera* NaoCamera::theInstance = 0;

NaoCamera::NaoCamera() :
  currentCamera(ImageInfo::lowerCamera),
  currentBuf(0),
  timeStamp(0)
{
  ASSERT(theInstance == 0);
  theInstance = this;

  ImageInfo::Camera current = currentCamera;
  ImageInfo::Camera other = currentCamera == ImageInfo::upperCamera ? ImageInfo::lowerCamera : ImageInfo::upperCamera;

  VERIFY(verifyNaoVersion());

  initSelectCamera(ImageInfo::upperCamera);
  initOpenVideoDevice();
  initSetCameraDefaults();
  initSelectCamera(ImageInfo::lowerCamera);
  initSetCameraDefaults();

  initSelectCamera(other);
  initRequestAndMapBuffers();
  initQueueAllBuffers();

  initResetCrop();
  initSetImageFormat();
  initSetFrameRate();
  initDefaultControlSettings();

  initSelectCamera(current);
  initResetCrop();
  initSetImageFormat();
  initSetFrameRate();
  initDefaultControlSettings();

  startCapturing();
}

NaoCamera::~NaoCamera()
{
  // disable streaming
  int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  VERIFY(ioctl(fd, VIDIOC_STREAMOFF, &type) != -1);

  // unmap buffers
  for(int i = 0; i < frameBufferCount; ++i)
#ifdef USE_USERPTR
    free(mem[i]);
#else
    munmap(mem[i], memLength[i]);
#endif

  // close the device
  close(fd);
  free(buf);

  theInstance = 0;
}

bool NaoCamera::captureNew()
{
  // requeue the buffer of the last captured image which is obsolete now
  if(currentBuf)
  {
    BH_TRACE;
    VERIFY(ioctl(fd, VIDIOC_QBUF, currentBuf) != -1);
  }
  BH_TRACE;

  struct pollfd pollfd = {fd, POLLIN | POLLPRI, 0};
  int polled = poll(&pollfd, 1, 500); // Fail after missing 15 frames (0.5s)
  if(polled < 0)
  {
    OUTPUT_ERROR("NaoCamera: Cannot poll. Reason: " << strerror(errno));
    ASSERT(false);
  }
  else if(polled == 0)
  {
    OUTPUT_ERROR("NaoCamera: 0.5 seconds passed and there's still no image to read from the camera. Terminating.");
    return false;
  }
  else if(pollfd.revents & (POLLERR | POLLNVAL))
  {
    OUTPUT_ERROR("NaoCamera: Polling failed.");
    return false;
  }
  // dequeue a frame buffer (this call blocks when there is no new image available) */
  VERIFY(ioctl(fd, VIDIOC_DQBUF, buf) != -1);
  timeStamp = SystemCall::getCurrentSystemTime();
  BH_TRACE;
  ASSERT(buf->bytesused == SIZE);
  currentBuf = buf;

  static bool shout = true;
  if(shout)
  {
    shout = false;
    printf("Camera is working\n");
  }

  return true;
}

const unsigned char* NaoCamera::getImage() const
{
#ifdef USE_USERPTR
  return currentBuf ? (unsigned char*)currentBuf->m.userptr : 0;
#else
  return currentBuf ? static_cast<unsigned char*>(mem[currentBuf->index]) : 0;
#endif
}

unsigned int NaoCamera::getTimeStamp() const
{
  ASSERT(currentBuf);
  return timeStamp;
}

int NaoCamera::getControlSetting(unsigned int id)
{
  struct v4l2_queryctrl queryctrl;
  queryctrl.id = id;
  if(ioctl(fd, VIDIOC_QUERYCTRL, &queryctrl) < 0)
    return -1;
  if(queryctrl.flags & V4L2_CTRL_FLAG_DISABLED)
    return -1; // not available
  if(queryctrl.type != V4L2_CTRL_TYPE_BOOLEAN && queryctrl.type != V4L2_CTRL_TYPE_INTEGER && queryctrl.type != V4L2_CTRL_TYPE_MENU)
    return -1; // not supported

  struct v4l2_control control_s;
  control_s.id = id;
  if(ioctl(fd, VIDIOC_G_CTRL, &control_s) < 0)
    return -1;
  if(control_s.value == queryctrl.default_value)
    return -1;
  return control_s.value;
}

bool NaoCamera::setControlSettings(std::list<CameraSettings::V4L2Setting> controlsettings)
{
  std::list<CameraSettings::V4L2Setting>::const_iterator it = controlsettings.begin();
  std::list<CameraSettings::V4L2Setting>::const_iterator end = controlsettings.end();
  bool success = true;
  for(; it != end; it++)
    if(!setControlSetting((*it).command, (*it).value))
      success = false;
  return success;
}

bool NaoCamera::setControlSetting(unsigned int id, int value)
{
  struct v4l2_queryctrl queryctrl;
  queryctrl.id = id;
  if(ioctl(fd, VIDIOC_QUERYCTRL, &queryctrl) < 0)
    return false;
  if(queryctrl.flags & V4L2_CTRL_FLAG_DISABLED)
    return false; // not available
  if(queryctrl.type != V4L2_CTRL_TYPE_BOOLEAN && queryctrl.type != V4L2_CTRL_TYPE_INTEGER && queryctrl.type != V4L2_CTRL_TYPE_MENU)
    return false; // not supported

  // clip value
  if(value < queryctrl.minimum)
    value = queryctrl.minimum;
  if(value > queryctrl.maximum)
    value = queryctrl.maximum;
  if(value < 0)
    value = queryctrl.default_value;

  struct v4l2_control control_s;
  control_s.id = id;
  control_s.value = value;
  if(ioctl(fd, VIDIOC_S_CTRL, &control_s) < 0)
    return false;
  return true;
}

void NaoCamera::setSettings(const CameraSettings& newset)
{
  if(settings == newset)
    return;

  std::list<CameraSettings::V4L2Setting> changes = settings.getChangesAndAssign(newset);

  ImageInfo::Camera current = getCurrentCamera();
  ImageInfo::Camera other = current == ImageInfo::lowerCamera ? ImageInfo::upperCamera : ImageInfo::lowerCamera;

  switchCamera(other);
  VERIFY(setControlSettings(changes));
  switchCamera(current);
  VERIFY(setControlSettings(changes));
}

ImageInfo::Camera NaoCamera::switchCamera(ImageInfo::Camera camera)
{
#ifndef NO_NAO_EXTENSIONS
  unsigned char cmd[2] = {camera, 0};
  int flip = camera == ImageInfo::upperCamera ? 1 : 0;
  int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

  // disable streaming
  VERIFY(ioctl(fd, VIDIOC_STREAMOFF, &type) != -1);

  // switch camera
  int i2cfd = openI2CAdapter();
  VERIFY(i2c_smbus_write_block_data(i2cfd, 220, 1, cmd) != -1);
  closeI2CAdapter(i2cfd);
  VERIFY(setControlSetting(V4L2_CID_VFLIP, flip));
  VERIFY(setControlSetting(V4L2_CID_HFLIP, flip));

  // enable streaming
  VERIFY(ioctl(fd, VIDIOC_STREAMON, &type) != -1);

  currentCamera = camera;
  return camera;
#else
  return ImageInfo::lowerCamera;
#endif // NO_NAO_EXTENSIONS
}

ImageInfo::Camera NaoCamera::switchToUpper()
{
  if(currentCamera == ImageInfo::upperCamera)
    return ImageInfo::upperCamera;

  return switchCamera(ImageInfo::upperCamera);
}

ImageInfo::Camera NaoCamera::switchToLower()
{
  if(currentCamera == ImageInfo::lowerCamera)
    return ImageInfo::lowerCamera;

  return switchCamera(ImageInfo::lowerCamera);
}

void NaoCamera::initSelectCamera(ImageInfo::Camera camera)
{
#ifndef NO_NAO_EXTENSIONS
  unsigned char cmd[2] = {camera, 0};
  int i2cfd = openI2CAdapter();
  VERIFY(i2c_smbus_write_block_data(i2cfd, 220, 1, cmd) != -1); // select camera
  closeI2CAdapter(i2cfd);
#endif
  currentCamera = camera;
}

void NaoCamera::initOpenVideoDevice()
{
  // open device
  fd = open("/dev/video0", O_RDWR);
  ASSERT(fd != -1);
}

void NaoCamera::initSetCameraDefaults()
{
  // set default parameters
#ifndef NO_NAO_EXTENSIONS
  struct v4l2_control control;
  memset(&control, 0, sizeof(control));
  control.id = V4L2_CID_CAM_INIT;
  control.value = 0;
  VERIFY(ioctl(fd, VIDIOC_S_CTRL, &control) >= 0);

  v4l2_std_id esid0 = WIDTH == 320 ? 0x04000000UL : 0x08000000UL;
  VERIFY(!ioctl(fd, VIDIOC_S_STD, &esid0));
#endif
}

void NaoCamera::initSetImageFormat()
{
  // set format
  struct v4l2_format fmt;
  memset(&fmt, 0, sizeof(struct v4l2_format));
  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  fmt.fmt.pix.width = WIDTH;
  fmt.fmt.pix.height = HEIGHT;
  fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
  fmt.fmt.pix.field = V4L2_FIELD_NONE;
  VERIFY(!ioctl(fd, VIDIOC_S_FMT, &fmt));

  ASSERT(fmt.fmt.pix.sizeimage == SIZE);
}

void NaoCamera::initSetFrameRate()
{
  // set frame rate
  struct v4l2_streamparm fps;
  memset(&fps, 0, sizeof(struct v4l2_streamparm));
  fps.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  VERIFY(!ioctl(fd, VIDIOC_G_PARM, &fps));
  fps.parm.capture.timeperframe.numerator = 1;
  fps.parm.capture.timeperframe.denominator = 30;
  VERIFY(ioctl(fd, VIDIOC_S_PARM, &fps) != -1);
}

void NaoCamera::initRequestAndMapBuffers()
{
  // request buffers
  struct v4l2_requestbuffers rb;
  memset(&rb, 0, sizeof(struct v4l2_requestbuffers));
  rb.count = frameBufferCount;
  rb.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
#ifdef USE_USERPTR
  rb.memory = V4L2_MEMORY_USERPTR;
#else
  rb.memory = V4L2_MEMORY_MMAP;
#endif
  VERIFY(ioctl(fd, VIDIOC_REQBUFS, &rb) != -1);
  ASSERT(rb.count == frameBufferCount);

  // map or prepare the buffers
  buf = static_cast<struct v4l2_buffer*>(calloc(1, sizeof(struct v4l2_buffer)));
#ifdef USE_USERPTR
  unsigned int bufferSize = SIZE;
  unsigned int pageSize = getpagesize();
  bufferSize = (bufferSize + pageSize - 1) & ~(pageSize - 1);
#endif
  for(int i = 0; i < frameBufferCount; ++i)
  {
#ifdef USE_USERPTR
    memLength[i] = bufferSize;
    mem[i] = memalign(pageSize, bufferSize);
#else
    buf->index = i;
    buf->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf->memory = V4L2_MEMORY_MMAP;
    VERIFY(ioctl(fd, VIDIOC_QUERYBUF, buf) != -1);
    memLength[i] = buf->length;
    mem[i] = mmap(0, buf->length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, buf->m.offset);
    ASSERT(mem[i] != MAP_FAILED);
#endif
  }
}

void NaoCamera::initQueueAllBuffers()
{
  // queue the buffers
  for(int i = 0; i < frameBufferCount; ++i)
  {
    buf->index = i;
    buf->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
#ifdef USE_USERPTR
    buf->memory = V4L2_MEMORY_USERPTR;
    buf->m.userptr = (unsigned long)mem[i];
    buf->length  = memLength[i];
#else
    buf->memory = V4L2_MEMORY_MMAP;
#endif
    VERIFY(ioctl(fd, VIDIOC_QBUF, buf) != -1);
  }
}

void NaoCamera::initDefaultControlSettings()
{
  // make sure automatic stuff is off
#ifndef NO_NAO_EXTENSIONS
  int flip = currentCamera == ImageInfo::upperCamera ? 1 : 0;
  VERIFY(setControlSetting(V4L2_CID_HFLIP, flip));
  VERIFY(setControlSetting(V4L2_CID_VFLIP, flip));

  std::list<CameraSettings::V4L2Setting> v4l2settings = settings.getInitSettings();
  VERIFY(setControlSettings(v4l2settings));
  SystemCall::sleep(300);
#else
  setControlSetting(V4L2_CID_AUTOEXPOSURE , 0);
  setControlSetting(V4L2_CID_AUTO_WHITE_BALANCE, 0);
  setControlSetting(V4L2_CID_AUTOGAIN, 0);
  setControlSetting(V4L2_CID_HUE_AUTO, 0);
  setControlSetting(V4L2_CID_EXPOSURE_AUTO, V4L2_EXPOSURE_MANUAL);
  setControlSetting(V4L2_CID_HFLIP, 0);
  setControlSetting(V4L2_CID_VFLIP, 0);
#endif
}

void NaoCamera::initResetCrop()
{
#ifndef NO_NAO_EXTENSIONS
  struct v4l2_cropcap cropcap;
  memset(&cropcap, 0, sizeof(cropcap));
  cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  VERIFY(ioctl(fd, VIDIOC_CROPCAP, &cropcap) != -1);

  struct v4l2_crop crop;
  memset(&crop, 0, sizeof(crop));
  crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  crop.c = cropcap.defrect;
  /* errno will be set to EINVAL if cropping is unsupported,
   * which would be fine, too */
  VERIFY(ioctl(fd, VIDIOC_S_CROP, &crop) != -1 || errno == EINVAL);
#endif
}

void NaoCamera::startCapturing()
{
  int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  VERIFY(ioctl(fd, VIDIOC_STREAMON, &type) != -1);
}

void NaoCamera::assertCameraSettings()
{
  bool allFine = true;
  // check frame rate
  struct v4l2_streamparm fps;
  memset(&fps, 0, sizeof(fps));
  fps.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  VERIFY(!ioctl(fd, VIDIOC_G_PARM, &fps));
  if(fps.parm.capture.timeperframe.numerator != 1)
  {
    OUTPUT(idText, text, "fps.parm.capture.timeperframe.numerator is wrong.");
    allFine = false;
  }
  if(fps.parm.capture.timeperframe.denominator != 30)
  {
    OUTPUT(idText, text, "fps.parm.capture.timeperframe.denominator is wrong.");
    allFine = false;
  }

  // check camera settings
  std::list<CameraSettings::V4L2Setting> v4l2settings = settings.getSettings();
  std::list<CameraSettings::V4L2Setting>::const_iterator it = v4l2settings.begin();
  std::list<CameraSettings::V4L2Setting>::const_iterator end = v4l2settings.end();
  for(; it != end; it++)
  {
    int value = getControlSetting((*it).command);
    if(value != (*it).value)
    {
      OUTPUT(idText, text, "Value for command " << (*it).command << " is " << value << " but should be " << (*it).value << ".");
      allFine = false;
    }
  }

  if(allFine)
  {
    OUTPUT(idText, text, "Camera settings match settings stored in hardware/driver.");
  }
}

void NaoCamera::writeCameraSettings()
{
  std::list<CameraSettings::V4L2Setting> v4l2settings = settings.getSettings();
  VERIFY(setControlSettings(v4l2settings));
}

int NaoCamera::openI2CAdapter()
{
#ifndef NO_NAO_EXTENSIONS
  int i2cfd = open("/dev/i2c-0", O_RDWR);
  ASSERT(i2cfd != -1);
  VERIFY(ioctl(i2cfd, 0x703, 8) == 0);
  return i2cfd;
#else
  return -1;
#endif
}

void NaoCamera::closeI2CAdapter(int filedes)
{
#ifndef NO_NAO_EXTENSIONS
  close(filedes);
#endif
}

bool NaoCamera::verifyNaoVersion()
{
  int i2cfd = openI2CAdapter();
  bool versionok = i2c_smbus_read_byte_data(i2cfd, 170) >= 2;
  closeI2CAdapter(i2cfd);
  return versionok;
}
