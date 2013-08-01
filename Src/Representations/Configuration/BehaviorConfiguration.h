/**
 * @file BehaviorConfiguration.h
 * Declaration of class BehaviorConfiguration
 *
 * @author Max Risler
 */

#pragma once

#include <string>
#include "Tools/Streams/Streamable.h"

class BehaviorConfiguration : public Streamable
{
public:
  BehaviorConfiguration() : agent("") {}

  std::string agent;

  virtual void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN();
    STREAM(agent);
    STREAM_REGISTER_FINISH();
  }
};
