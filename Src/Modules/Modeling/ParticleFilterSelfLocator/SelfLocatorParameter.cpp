#include "SelfLocatorParameter.h"
#include "Platform/BHAssert.h"
#include "Tools/Streams/InStreams.h"
#include "Tools/Global.h"

SelfLocatorParameter::SelfLocatorParameter()
{
  load("selfloc.cfg");
}

void SelfLocatorParameter::load(const std::string& fileName)
{
  InConfigMap file(Global::getSettings().expandLocationFilename(fileName));
  ASSERT(file.exists());
  file >> *this;
}
