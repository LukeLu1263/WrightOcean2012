/**
* @file BH2011BehaviorLEDSymbols.cpp
* Implementation of class BH2011BehaviorLEDSymbols.
* @author jeff
*/

#include "BH2011BehaviorLEDSymbols.h"

void BH2011BehaviorLEDSymbols::registerSymbols(xabsl::Engine& engine)
{
  engine.registerEnumElement("behavior_led_request.color", "behavior_led_request.color.red", BehaviorLEDRequest::red);
  engine.registerEnumElement("behavior_led_request.color", "behavior_led_request.color.green", BehaviorLEDRequest::green);
  engine.registerEnumElement("behavior_led_request.color", "behavior_led_request.color.blue", BehaviorLEDRequest::blue);
  engine.registerEnumElement("behavior_led_request.color", "behavior_led_request.color.white", BehaviorLEDRequest::white);
  engine.registerEnumElement("behavior_led_request.color", "behavior_led_request.color.magenta", BehaviorLEDRequest::magenta);
  engine.registerEnumElement("behavior_led_request.color", "behavior_led_request.color.yellow", BehaviorLEDRequest::yellow);

  engine.registerEnumElement("behavior_led_request.state", "behavior_led_request.state.off", LEDRequest::off);
  engine.registerEnumElement("behavior_led_request.state", "behavior_led_request.state.on", LEDRequest::on);
  engine.registerEnumElement("behavior_led_request.state", "behavior_led_request.state.blinking", LEDRequest::blinking);
  engine.registerEnumElement("behavior_led_request.state", "behavior_led_request.state.fastBlinking", LEDRequest::fastBlinking);
  engine.registerEnumElement("behavior_led_request.state", "behavior_led_request.state.half", LEDRequest::half);

  engine.registerEnumeratedOutputSymbol("behavior_led_request.leftEye", "behavior_led_request.state", (int*)&behaviorLEDRequest.modifiers[BehaviorLEDRequest::leftEye]);
  engine.registerEnumeratedOutputSymbol("behavior_led_request.rightEye", "behavior_led_request.state", (int*)&behaviorLEDRequest.modifiers[BehaviorLEDRequest::rightEye]);
  engine.registerEnumeratedOutputSymbol("behavior_led_request.leftEar", "behavior_led_request.state", (int*)&behaviorLEDRequest.modifiers[BehaviorLEDRequest::leftEar]);
  engine.registerEnumeratedOutputSymbol("behavior_led_request.rightEar", "behavior_led_request.state", (int*)&behaviorLEDRequest.modifiers[BehaviorLEDRequest::rightEar]);

  engine.registerEnumeratedOutputSymbol("behavior_led_request.leftEyeColor", "behavior_led_request.color", (int*)&behaviorLEDRequest.leftEyeColor);
  engine.registerEnumeratedOutputSymbol("behavior_led_request.rightEyeColor", "behavior_led_request.color", (int*)&behaviorLEDRequest.rightEyeColor);
}

void BH2011BehaviorLEDSymbols::update()
{
  // Reset all LEDs:
  for(int i = 0; i < BehaviorLEDRequest::numOfBehaviorLEDs; ++i)
    behaviorLEDRequest.modifiers[i] = LEDRequest::on;

  behaviorLEDRequest.leftEyeColor = BehaviorLEDRequest::default_color;
  behaviorLEDRequest.rightEyeColor = BehaviorLEDRequest::default_color;
}

