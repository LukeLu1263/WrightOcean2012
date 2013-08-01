#pragma once

#ifndef TARGET_TOOL
#include "Tools/Debugging/Debugging.h"
#endif

namespace bs_Debug
{
  void error(const std::string &msg);
  void warning(const std::string &msg);
  void info(const std::string &msg);
  void infoi(const std::string &msg, int i);
  void bs_assert(bool b, const std::string &msg);
}

