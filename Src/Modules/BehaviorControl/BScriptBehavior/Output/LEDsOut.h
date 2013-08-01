#pragma once

#include "Representations/BehaviorControl/BehaviorLEDRequest.h"

class LEDsOut
{
public:
  LEDsOut()
    : request(0)
  {}

  void init(BehaviorLEDRequest& request)
  {
    this->request = &request;
  }

  void setState(int led, int state)
  {
    ASSERT(led >= 0 && led < BehaviorLEDRequest::numOfBehaviorLEDs);
    ASSERT(state >= 0 && state < LEDRequest::numOfLEDStates);
    ASSERT(request);

    request->modifiers[led] = (LEDRequest::LEDState)state;
  }

  void setLeftEyeColor(int color)
  {
    request->leftEyeColor = (BehaviorLEDRequest::EyeColor)color;
  }

  void setRightEyeColor(int color)
  {
    request->rightEyeColor = (BehaviorLEDRequest::EyeColor)color;
  }

private:
  BehaviorLEDRequest* request;
};
