#include <stdlib.h>
#include <stdio.h>

#include "src/bScriptEngine.h"
#include "src/codeGenerator/dotGraphGenerator.h"
#include "src/codeGenerator/cppCodeGenerator.h"
#include "Modules/BehaviorControl/BScriptBehavior/BSModules/bs_BH.h"

void usage(const char* name)
{
  printf("usage: %s <b-script-main-module-filename> <entry-function> <outputDir>\n", name);
}

int main(int argc, char** argv)
{
  if(argc <= 3)
  {
    usage(argv[0]);
    return EXIT_FAILURE;
  }

  const char* mainModulefilename = argv[1];

  BScriptEngine engine;
  //load builtins
  if(!engine.init())
    return EXIT_FAILURE;

  //set input and output
  BehaviorInput input;
  BehaviorOutput output;
  if(!engine.requires("BH"))
    return EXIT_FAILURE;
  if(!engine.setInput(&input))
    return EXIT_FAILURE;
  if(!engine.setOutput(&output))
    return EXIT_FAILURE;

  //mainCallCode = requires "<moduleFileName>"
  //               func main()
  //                   <moduleName>.<funcName>()
  std::string mainCallCode;
  mainCallCode += "requires \"";
  mainCallCode += mainModulefilename;
  mainCallCode += "\"\n"
                  "func main()\n"
                  "   " + engine.getModuleName(mainModulefilename) + "::" + argv[2] + "()\n";

  if(!engine.setMainCallCode(mainCallCode))
    return EXIT_FAILURE;
  {
    try
    {
      CppCodeGenerator cppCodeGen(argv[3]);
      engine.generateCode(cppCodeGen);
    }
    catch(TreePrinter::TreePrinterException& e)
    {
      fprintf(stderr, "%s\n", e.what());
      return EXIT_FAILURE;
    }
  }

  {
    std::string filename(argv[3]);
    filename += "/call.dot";
    DotGraphGenerator dotGen(filename);
    engine.generateCode(dotGen);
  }

  return EXIT_SUCCESS;
}

