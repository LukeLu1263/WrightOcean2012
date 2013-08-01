/**
* @file ImageInfo.h
* A class representing information on images.
* @author Felix Wenk
*/

#pragma once

/**
* @class ImageInfo
* A class representing information on images.
*/
class ImageInfo : public Streamable
{
private:
  void serialize(In* in, Out* out)
  {
    unsigned char prevCamera = (unsigned char) this->prevCamera;
    unsigned char camera = (unsigned char) this->camera;
    STREAM_REGISTER_BEGIN();
    STREAM(prevCamera);
    STREAM(camera);
    STREAM_REGISTER_FINISH();
    this->prevCamera = (Camera) prevCamera;
    this->camera = (Camera) camera;
  }

public:
  /**
   * @enum Camera
   * Enum representing the possible sources of an image.
   */
  enum Camera { upperCamera = 0x01, lowerCamera = 0x02 };

  Camera prevCamera;
  Camera camera;

  ImageInfo() : prevCamera(lowerCamera), camera(lowerCamera) {}
  bool fromLowerCamera() const  { return camera == lowerCamera; }
  bool fromLowerPrevCamera() const { return prevCamera == lowerCamera; }
  bool cameraChanged() const { return camera != prevCamera; }
};

/**
 * @class ImageRequest
 * Class representing a request to switch (or not to switch) the camera.
 *
 * This class is complementary to ImageInfo
 */
class ImageRequest : public Streamable
{
private:
  void serialize(In* in, Out* out)
  {
    unsigned char requestedCamera = (unsigned char) this->requestedCamera;
    STREAM_REGISTER_BEGIN();
    STREAM(requestedCamera);
    STREAM_REGISTER_FINISH();
    this->requestedCamera = (ImageInfo::Camera) requestedCamera;
  }

public:
  ImageInfo::Camera requestedCamera;

  ImageRequest() : requestedCamera(ImageInfo::lowerCamera) {}
};
