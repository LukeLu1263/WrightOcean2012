/**
* @file CognitionConfigurationDataProvider.cpp
* This file implements a module that provides data loaded from configuration files.
* @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
*/

#include <cstdio>

#include "CognitionConfigurationDataProvider.h"
#include "Tools/Configuration/ConfigMap.h"
#include "Tools/Settings.h"
#include "Platform/File.h"

PROCESS_WIDE_STORAGE(CognitionConfigurationDataProvider) CognitionConfigurationDataProvider::theInstance = 0;

CognitionConfigurationDataProvider::CognitionConfigurationDataProvider() :
  theFieldDimensions(0),
  theCameraSettings(0),
  theCameraCalibration(0),
  theColorTable64(0),
  theRobotDimensions(0),
  theBehaviorConfiguration(0),
  theDamageConfiguration(0)
{
  theInstance = this;

  readFieldDimensions();
  readCameraSettings();
  readCameraCalibration();
  readColorTable64();
  readRobotDimensions();

  readBehaviorConfiguration();
  readDamageConfiguration();
}

CognitionConfigurationDataProvider::~CognitionConfigurationDataProvider()
{
  if(theFieldDimensions)
    delete theFieldDimensions;
  if(theCameraSettings)
    delete theCameraSettings;
  if(theCameraCalibration)
    delete theCameraCalibration;
  if(theColorTable64)
    delete theColorTable64;
  if(theBehaviorConfiguration)
    delete theBehaviorConfiguration;
  if(theDamageConfiguration)
    delete theDamageConfiguration;
  theInstance = 0;
}

void CognitionConfigurationDataProvider::update(FieldDimensions& fieldDimensions)
{
  if(theFieldDimensions)
  {
    fieldDimensions = *theFieldDimensions;
    delete theFieldDimensions;
    theFieldDimensions = 0;
  }
}

void CognitionConfigurationDataProvider::update(CameraSettings& cameraSettings)
{
  if(theCameraSettings)
  {
    cameraSettings = *theCameraSettings;
    delete theCameraSettings;
    theCameraSettings = 0;
  }
}

void CognitionConfigurationDataProvider::update(CameraCalibration& cameraCalibration)
{
  if(theCameraCalibration)
  {
    cameraCalibration = *theCameraCalibration;
    delete theCameraCalibration;
    theCameraCalibration = 0;
  }
}

void CognitionConfigurationDataProvider::update(ColorTable64& colorTable64)
{
  if(theColorTable64)
  {
    colorTable64 = *theColorTable64;
    delete theColorTable64;
    theColorTable64 = 0;
  }
  DEBUG_RESPONSE("module:CognitionConfigurationDataProvider:hashct",
  {
    OUTPUT(idText, text, "colortable hash: " << colorTable64.hash());
  });
}

void CognitionConfigurationDataProvider::update(RobotDimensions& robotDimensions)
{
  if(theRobotDimensions)
  {
    robotDimensions = *theRobotDimensions;
    delete theRobotDimensions;
    theRobotDimensions = 0;
  }
}

void CognitionConfigurationDataProvider::update(BehaviorConfiguration& behaviorConfiguration)
{
  if(theBehaviorConfiguration)
  {
    behaviorConfiguration = *theBehaviorConfiguration;
    delete theBehaviorConfiguration;
    theBehaviorConfiguration = 0;
  }
}

void CognitionConfigurationDataProvider::update(DamageConfiguration& damageConfiguration)
{
  if(theDamageConfiguration)
  {
    damageConfiguration = *theDamageConfiguration;
    delete theDamageConfiguration;
    theDamageConfiguration = 0;
  }
}

void CognitionConfigurationDataProvider::readFieldDimensions()
{
  ASSERT(!theFieldDimensions);

  InConfigFile stream(Global::getSettings().expandLocationFilename("field.cfg"));
  if(!stream.exists())
    return;
  theFieldDimensions = new FieldDimensions;
  theFieldDimensions->load();
}

void CognitionConfigurationDataProvider::readCameraSettings()
{
  ASSERT(!theCameraSettings);

  InConfigMap* stream = new InConfigMap(Global::getSettings().expandRobotLocationFilename("camera.cfg"));
  if(!stream->exists())
  {
    delete stream;
    stream = new InConfigMap(Global::getSettings().expandLocationFilename("camera.cfg"));
  }

  if(stream->exists())
  {
    theCameraSettings = new CameraSettings;
    *stream >> *theCameraSettings;
  }

  delete stream;
}

void CognitionConfigurationDataProvider::readCameraCalibration()
{
  ASSERT(!theCameraCalibration);

  InConfigMap stream(Global::getSettings().expandRobotFilename("cameraCalibration.cfg"));
  if(stream.exists())
  {
    theCameraCalibration = new CameraCalibration;
    stream >> *theCameraCalibration;
  }
}

void CognitionConfigurationDataProvider::readColorTable64()
{
  ASSERT(!theColorTable64);

  std::string ctName = "coltable";
  const CameraCalibration& cameraCalibration = theCameraCalibration ? *theCameraCalibration : CognitionConfigurationDataProviderBase::theCameraCalibration;
  if(cameraCalibration.colorTemperature > CameraCalibration::defaultCamera)
  {
    std::string ext = CameraCalibration::getName(cameraCalibration.colorTemperature);
    ext[0] = toupper(ext[0]);
    ctName += ext;
  }
  ctName += ".c64";

  std::string fileName = Global::getSettings().expandRobotLocationFilename(ctName);
  InBinaryFile* stream = new InBinaryFile(fileName);
  if(!stream->exists())
  {
    delete stream;
    std::string newFileName = Global::getSettings().expandLocationFilename(ctName);
    printf("Cannot read \"%s\"\n\t trying \"%s\"\n", fileName.c_str(), newFileName.c_str());
    fileName = newFileName;
    stream = new InBinaryFile(fileName);
  }
  if(!stream->exists())
  {
    delete stream;
    std::string newFileName = Global::getSettings().expandRobotLocationFilename("coltable.c64");
    printf("Cannot read \"%s\"\n\t trying \"%s\"\n", fileName.c_str(), newFileName.c_str());
    fileName = newFileName;
    stream = new InBinaryFile(fileName);
  }
  if(!stream->exists())
  {
    delete stream;
    std::string newFileName = Global::getSettings().expandLocationFilename("coltable.c64");
    printf("Cannot read \"%s\"\n\t trying \"%s\"\n", fileName.c_str(), newFileName.c_str());
    fileName = newFileName;
    stream = new InBinaryFile(fileName);
  }

  if(stream->exists())
  {
    theColorTable64 = new ColorTable64;
    *stream >> *theColorTable64;
    printf("colortable hash: \"%s\"\n", theColorTable64->hash().c_str());
  }

  delete stream;
}


void CognitionConfigurationDataProvider::readRobotDimensions()
{
  ASSERT(!theRobotDimensions);

  InConfigMap stream(Global::getSettings().expandRobotFilename("robotDimensions.cfg"));
  if(stream.exists())
  {
    theRobotDimensions = new RobotDimensions;
    stream >> *theRobotDimensions;
  }
}

void CognitionConfigurationDataProvider::readBehaviorConfiguration()
{
  ASSERT(!theBehaviorConfiguration);

  InConfigMap stream(Global::getSettings().expandLocationFilename("behavior.cfg"));
  if(stream.exists())
  {
    theBehaviorConfiguration = new BehaviorConfiguration;
    stream >> *theBehaviorConfiguration;
  }
}

void CognitionConfigurationDataProvider::readDamageConfiguration()
{
  ASSERT(!theDamageConfiguration);

  InConfigMap stream(Global::getSettings().expandRobotFilename("damageConfiguration.cfg"));
  if(stream.exists())
  {
    theDamageConfiguration = new DamageConfiguration;
    stream >> *theDamageConfiguration;
  }
}

bool CognitionConfigurationDataProvider::handleMessage(InMessage& message)
{
  CognitionConfigurationDataProvider* instrance = theInstance;
  return instrance && instrance->handleMessage2(message);
}

bool CognitionConfigurationDataProvider::handleMessage2(InMessage& message)
{
  switch(message.getMessageID())
  {
  case idColorTable64:
    if(!theColorTable64)
      theColorTable64 = new ColorTable64;
    message.bin >> *theColorTable64;
    return true;

  case idWriteColorTable64:
    if(!theColorTable64)
      theColorTable64 = new ColorTable64;
    message.bin >> *theColorTable64;
    {
      std::string ctName = "coltable";
      const CameraCalibration& cameraCalibration = theCameraCalibration ? *theCameraCalibration : CognitionConfigurationDataProviderBase::theCameraCalibration;
      if(cameraCalibration.colorTemperature > CameraCalibration::defaultCamera)
      {
        std::string ext = CameraCalibration::getName(cameraCalibration.colorTemperature);
        ext[0] = toupper(ext[0]);
        ctName += ext;
      }
      ctName += ".c64";
      OutBinaryFile stream(Global::getSettings().expandRobotLocationFilename(ctName));
      if(stream.exists())
      {
        stream << *theColorTable64;
      }
      else
      {
        OutBinaryFile stream(Global::getSettings().expandLocationFilename(ctName));
        stream << *theColorTable64;
      }
    }
    return true;

  default:
    return false;
  }
}

MAKE_MODULE(CognitionConfigurationDataProvider, Infrastructure)
