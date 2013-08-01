#include "../Util/b-script/src/bsNativeModule.h"
#include "bs_Debug.h"

namespace bs_Debug
{
  void error(const std::string &errMsg)
  {
#ifndef TARGET_TOOL
    std::string msg("BehaviorError: ");
    msg += errMsg;
    OUTPUT_ERROR(msg.c_str());
#endif
  }

  void warning(const std::string &msg)
  {
#ifndef TARGET_TOOL
    OUTPUT_WARNING("BehaviorWarning: " << msg);
#endif
  }

  void info(const std::string &msg)
  {
#ifndef TARGET_TOOL
    OUTPUT(idText, text, "BehaviorInfo: " << msg);
#endif
  }

  void infoi(const std::string &msg, int i)
  {
#ifndef TARGET_TOOL
    OUTPUT(idText, text, "BehaviorInfo: " << msg << " => " << i);
#endif
  }

  void bs_assert(bool b, const std::string &msg)
  {
#ifndef TARGET_TOOL
    if(!b)
    {
      ASSERT(false);
      OUTPUT_ERROR("BehaviorAssert failed: " << msg);
    }
#endif
  }
}

BS_MODULE(Debug,
  registerFunctionVoid(module, "error", &bs_Debug::error);
  registerFunctionVoid(module, "warning", &bs_Debug::warning);
  registerFunctionVoid(module, "info", &bs_Debug::info);
  registerFunctionVoid(module, "infoi", &bs_Debug::infoi);
  registerFunctionVoid(module, "assert", &bs_Debug::bs_assert, "bs_Debug::bs_assert");
  );

