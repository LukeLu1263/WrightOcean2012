/**
* @file BScriptBehaviorEngine.cpp
* Implementation of class BScriptBehaviorEngine.
* @author jeff
*/

#include "Platform/File.h"
#include "Tools/Debugging/Debugging.h"

#include "BScriptBehaviorEngine.h"
#include "../Util/b-script/src/bsNativeModule.h"
#include "../Util/b-script/src/codeGenerator/cppCodeGenerator.h"
#include "../Util/b-script/src/codeGenerator/debugTreeGenerator.h"

void bScriptErrorHandler(const char* format, ...)
{
#define bufsize 512
  char buf[bufsize + 1];

  va_list ap;
  va_start(ap, format);
  vsnprintf(buf, bufsize, format, ap);
  va_end(ap);

  OUTPUT_ERROR("b-script ERROR: " << buf);
}

BScriptBehaviorEngine::BScriptBehaviorEngine()
  : engine(std::string(File::getBHDir()) + "/Config/Behavior/", &bScriptErrorHandler),
    engineInitialized(false),
    inputRepresentations(0)
#ifndef RELEASE
    , useModuleLibrary(true),
    lastUseModuleLibrary(true)
#endif
{
}

BScriptBehaviorEngine::~BScriptBehaviorEngine()
{
  if(inputRepresentations)
  {
    delete inputRepresentations;
    inputRepresentations = 0;
  }
}

int BScriptBehaviorEngine::drawCallTree(const CallTreeNode& node, std::string indent, int yPos)
{
  yPos -= 150;
  if(node.type == CallTreeNode::root)
    yPos += 150; //skip root node
  else if(node.type == CallTreeNode::call)
    DRAWTEXT("module:BScriptBehaviorEngine:callTree",
             -2000,
             yPos,
             1000,
             ColorClasses::black,
             indent.c_str() << "->" << node.name.c_str());
  else if(node.type == CallTreeNode::trace)
    DRAWTEXT("module:BScriptBehaviorEngine:callTree",
             -2000,
             yPos,
             1000,
             ColorClasses::black,
             indent.c_str() << "| " << node.name.c_str());
  else
    ASSERT(false);

  indent += "    ";
  for(std::vector<CallTreeNode>::const_iterator i = node.childs.begin(); i != node.childs.end(); i++)
    yPos = drawCallTree(*i, indent, yPos);

  return yPos;
}

void BScriptBehaviorEngine::init()
{
  if(inputRepresentations)
  {
    delete inputRepresentations;
    inputRepresentations = 0;
  }

  inputRepresentations  = new InputRepresentations(theFrameInfo,
                                                   theGameInfo,
                                                   theRobotInfo,
                                                   theOwnTeamInfo,
                                                   theOpponentTeamInfo,
                                                   theFieldDimensions,
                                                   theRobotPose,
                                                   theBallModel,
                                                   theBallHypotheses,
                                                   theObstacleModel,
                                                   theRobotsModel,
                                                   theMotionInfo,
                                                   theFallDownState,
                                                   theKickInfo,
                                                   theKeyStates,
                                                   theGoalPercept,
                                                   theRobotModel,
                                                   theRobotDimensions,
                                                   theTorsoMatrix);
  output.init(*inputRepresentations);
  input.init(*inputRepresentations);

  engineInitialized = initEngine();

  if(engineInitialized)
    OUTPUT(idText, text, "b-script: Engine initialized");
}

bool BScriptBehaviorEngine::initEngine()
{
  if(!engine.init())
    return false;

  if(!engine.requires("BH"))
    return false;

  if(!engine.setOutput(&output))
    return false;

  if(!engine.setInput(&input))
    return false;

#ifndef WIN32
  //no module library support for windows =|
#ifdef TARGET_ROBOT
  const char* libraryName = "/Config/libb-scriptBehavior.so";
#elif defined(MACOSX)
  const char* libraryName = "/Build/b-scriptBehaviorSim/MacOS/Release/b-scriptBehaviorSim.dylib";
#else
  const char* libraryName = "/Build/b-scriptBehaviorSim/libb-scriptBehaviorSim.so";
#endif

#ifndef RELEASE
  if(useModuleLibrary)
#endif //release
    if(!engine.loadModuleLibrary(std::string(File::getBHDir()) + libraryName, false))
      return false;
#endif //win32

#ifdef RELEASE
  std::string
#endif
  mainCallCode = "requires \"Behavior.bs\"\n"
                 "func main()\n"
                 "   Behavior::main()\n";

  if(!engine.setMainCallCode(mainCallCode))
    return false;

#ifndef RELEASE
  lastMainCallCode = mainCallCode;
#endif

  return true;
}

void BScriptBehaviorEngine::update(HeadMotionRequest& headMotionRequest)
{
  //debug stuff
  DECLARE_DEBUG_DRAWING("module:BScriptBehaviorEngine:callTree", "drawingOnField");
  DEBUG_RESPONSE_ONCE("module:BScriptBehaviorEngine:reloadBehavior", init(););

  //dummy debug responses to avoid Syntax Errors when loading the BH Scenes
  DEBUG_RESPONSE("automatedRequests:xabsl:debugSymbols", ;);
  DEBUG_RESPONSE("automatedRequests:xabsl:debugMessages", ;);

#ifndef RELEASE
  MODIFY("module:BScriptBehaviorEngine:main", mainCallCode);
  MODIFY("module:BScriptBehaviorEngine:useModuleLibrary", useModuleLibrary);
  if(lastMainCallCode != mainCallCode)
  {
    engine.setMainCallCode(mainCallCode);
    lastMainCallCode = mainCallCode;
  }

  if(lastUseModuleLibrary != useModuleLibrary)
  {
    init();
    lastUseModuleLibrary = useModuleLibrary;
  }
#endif

  //execute engine
  ASSERT(inputRepresentations);
  input.update(*inputRepresentations);
  output.reset(*inputRepresentations);

  if(engineInitialized)
    VERIFY(engine.execute());
  else
  {
    output.behaviorLEDRequest.leftEyeColor = BehaviorLEDRequest::magenta;
    output.behaviorLEDRequest.rightEyeColor = BehaviorLEDRequest::magenta;
    for(int i = 0; i < BehaviorLEDRequest::numOfBehaviorLEDs; ++i)
      output.behaviorLEDRequest.modifiers[i] = LEDRequest::blinking;
  }
  COMPLEX_DRAWING("module:BScriptBehaviorEngine:callTree", drawCallTree(engine.getCallTree()););
  //end execute engine

  //copy headMotionRequest
  headMotionRequest = output.headMotionRequest;
}

void BScriptBehaviorEngine::update(MotionRequest& motionRequest)
{
  motionRequest = output.motionRequest;
}

void BScriptBehaviorEngine::update(BehaviorLEDRequest& behaviorLEDRequest)
{
  behaviorLEDRequest = output.behaviorLEDRequest;
}

void BScriptBehaviorEngine::update(SoundRequest& soundRequest)
{
  soundRequest = output.soundRequest;
}

void BScriptBehaviorEngine::update(BehaviorControlOutput& behaviorControlOutput)
{
  behaviorControlOutput.behaviorData.role = (BehaviorData::Role)theRobotInfo.number;

  behaviorControlOutput.gameInfo = theGameInfo;
  behaviorControlOutput.gameInfo.state = output.gameInfo.state;
  behaviorControlOutput.gameInfo.secondaryState = output.gameInfo.secondaryState;
  behaviorControlOutput.gameInfo.kickOffTeam = output.gameInfo.kickOffTeam;

  behaviorControlOutput.robotInfo = theRobotInfo;
  behaviorControlOutput.robotInfo.penalty = output.robotInfo.penalty;

  behaviorControlOutput.ownTeamInfo = theOwnTeamInfo;
  behaviorControlOutput.ownTeamInfo.teamColour = output.teamInfo.color;
}

MAKE_MODULE(BScriptBehaviorEngine, Behavior Control)

