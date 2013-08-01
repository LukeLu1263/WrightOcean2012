/**
* @file CameraProvider.cpp
* This file declares a module that provides camera images.
* @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
*/

#include <cstdio>

#include "CameraProvider.h"
#include "Representations/Perception/JPEGImage.h"
#include "Platform/Camera.h"
#include "Platform/SystemCall.h"

PROCESS_WIDE_STORAGE(CameraProvider) CameraProvider::theInstance = 0;

CameraProvider::CameraProvider()
{
#ifdef CAMERA_INCLUDED
  camera = new NaoCamera();
#else
  camera = NULL;
#endif
  theInstance = this;
}

CameraProvider::~CameraProvider()
{
#ifdef CAMERA_INCLUDED
  if(camera)
    delete camera;
#endif
  theInstance = 0;
}

void CameraProvider::update(Image& image)
{
#ifdef CAMERA_INCLUDED
  ASSERT(camera->getImage());
  image.setImage(const_cast<unsigned char*>(camera->getImage()));
  image.timeStamp = camera->getTimeStamp();
  camera->setSettings(theCameraSettings);
  DEBUG_RESPONSE("module:CameraProvider:assertCameraSettings", camera->assertCameraSettings(););
  DEBUG_RESPONSE("module:CameraProvider:writeCameraSettings", camera->writeCameraSettings(););
#endif // CAMERA_INCLUDED
  DEBUG_RESPONSE("representation:JPEGImage", OUTPUT(idJPEGImage, bin, JPEGImage(image)););
}

void CameraProvider::update(ImageInfo& imageInfo)
{
#ifdef CAMERA_INCLUDED
  imageInfo.prevCamera = imageInfo.camera;
  imageInfo.camera = camera->getCurrentCamera();
  if(theImageRequest.requestedCamera != imageInfo.camera)
  {
    camera->switchCamera(theImageRequest.requestedCamera);
    imageInfo.camera = camera->getCurrentCamera();
  }
#endif
  return;
}

void CameraProvider::update(FrameInfo& frameInfo)
{
  frameInfo.time = theImage.timeStamp;
}

void CameraProvider::update(CognitionFrameInfo& cognitionFrameInfo)
{
  cognitionFrameInfo.time = theImage.timeStamp;
}

bool CameraProvider::isFrameDataComplete()
{
#ifdef CAMERA_INCLUDED
  if(theInstance)
    return theInstance->camera->getImage() ? true : false;
  else
#endif
    return true;
}

void CameraProvider::waitForFrameData()
{
#ifdef CAMERA_INCLUDED
  static bool reset = false;
  DEBUG_RESPONSE("module:CameraProvider:resetCamera", reset = true;);
  if((theInstance && !theInstance->camera->captureNew()) || reset)
  {
    OUTPUT_WARNING("CameraProvider: Capturing image failed. Resetting camera.");
    delete theInstance->camera;
    theInstance->camera = NULL;
    theInstance->camera = new NaoCamera();
    reset = false;
  }
#endif
}

MAKE_MODULE(CameraProvider, Infrastructure)
