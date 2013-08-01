/**
* @file BHXabslMessage.h
*
* Definition of class BHXabslMessage.
*
* @author Max Risler
*/

#pragma once

#include "Tools/Xabsl/XabslEngine/XabslTeamMessage.h"
#include "Tools/Streams/Streamable.h"
#include "Platform/BHAssert.h"

class BHXabslMessage : public xabsl::TeamMessage {};

class BHXabslMessageCompressed : public Streamable
{
  std::vector<unsigned char> coopStatesExecuted;
  std::vector<unsigned char> coopStatesEntering;
  std::vector<unsigned char> coopStatesOptionExecuted;
  unsigned char agentPriority;
public:
  void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN();
    STREAM(coopStatesExecuted);
    STREAM(coopStatesEntering);
    STREAM(coopStatesOptionExecuted);
    STREAM(agentPriority);
    STREAM_REGISTER_FINISH();
  }

  BHXabslMessageCompressed(const BHXabslMessage& message)
    : agentPriority(static_cast<unsigned char>(message.agentPriority))
  {
    for(int i = 0; i < message.coopStatesExecuted.getSize(); i++)
    {
      ASSERT(message.coopStatesExecuted[i] >= 0 && message.coopStatesExecuted[i] <= 255);
      coopStatesExecuted.push_back(static_cast<unsigned char>(message.coopStatesExecuted[i]));
    }
    for(int i = 0; i < message.coopStatesEntering.getSize(); i++)
    {
      ASSERT(message.coopStatesEntering[i] >= 0 && message.coopStatesEntering[i] <= 255);
      coopStatesEntering.push_back(static_cast<unsigned char>(message.coopStatesEntering[i]));
    }
    for(int i = 0; i < message.coopStatesOptionExecuted.getSize(); i++)
    {
      ASSERT(message.coopStatesOptionExecuted[i] >= 0 && message.coopStatesOptionExecuted[i] <= 255);
      coopStatesOptionExecuted.push_back(static_cast<unsigned char>(message.coopStatesOptionExecuted[i]));
    }
  }

  BHXabslMessage unpack()
  {
    BHXabslMessage message;
    for(size_t i = 0; i < coopStatesExecuted.size(); i++)
    {
      message.coopStatesExecuted.append(static_cast<int>(coopStatesExecuted[i]));
    }
    for(size_t i = 0; i < coopStatesEntering.size(); i++)
    {
      message.coopStatesEntering.append(static_cast<int>(coopStatesEntering[i]));
    }
    for(size_t i = 0; i < coopStatesOptionExecuted.size(); i++)
    {
      message.coopStatesOptionExecuted.append(static_cast<int>(coopStatesOptionExecuted[i]));
    }
    message.agentPriority = static_cast<int>(agentPriority);
    return message;
  }
};

