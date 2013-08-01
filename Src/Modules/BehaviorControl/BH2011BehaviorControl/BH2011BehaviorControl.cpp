/**
* @file BH2011BehaviorControl.cpp
*
* Implementation of class BH2011BehaviorControl.
*
* @author Max Risler
* @author Judith Müller
*/

#include "BH2011BehaviorControl.h"
#include "Tools/Debugging/ReleaseOptions.h"
#include "Modules/BehaviorControl/CommonSymbols/CommonBIKESymbols.h"
#include "Modules/BehaviorControl/CommonSymbols/KeySymbols.h"
#include "Modules/BehaviorControl/CommonSymbols/MathSymbols.h"
#include "Modules/BehaviorControl/CommonSymbols/MotionSymbols.h"
#include "Modules/BehaviorControl/CommonSymbols/SoundSymbols.h"
#include "Symbols/BH2011BallSymbols.h"
#include "Symbols/BH2011BIKESymbols.h"
#include "Symbols/BH2011FallDownSymbols.h"
#include "Symbols/BH2011FieldSymbols.h"
#include "Symbols/BH2011GameSymbols.h"
#include "Symbols/BH2011GoalSymbols.h"
#include "Symbols/BH2011HeadSymbols.h"
#include "Symbols/BH2011BehaviorLEDSymbols.h"
#include "Symbols/BH2011LocatorSymbols.h"
#include "Symbols/BH2011ObstacleSymbols.h"
#include "Symbols/BH2011TeamSymbols.h"
#include "Symbols/BH2011RoleSymbols.h"
#include "Symbols/BH2011SoccerSymbols.h"
PROCESS_WIDE_STORAGE(BH2011BehaviorControl) BH2011BehaviorControl::theInstance = 0;

BH2011BehaviorControl::BH2011BehaviorControl()
  : BHXabslEngineExecutor("bh2011", theFrameInfo.time), syncTicks(15.f)
{
  theInstance = this;

  // Common Symbols
  symbols.push_back(new CommonBIKESymbols(behaviorControlOutput.motionRequest, theBikeEngineOutput, theMotionInfo));
  symbols.push_back(new KeySymbols(theKeyStates));
  symbols.push_back(new MathSymbols());
  symbols.push_back(new MotionSymbols(behaviorControlOutput.motionRequest, theMotionInfo, theRobotInfo));
  symbols.push_back(new SoundSymbols(behaviorControlOutput.soundRequest));

  // Symbols
  symbols.push_back(new BH2011BallSymbols(theRobotInfo, theBallModel, theBallHypotheses, theFrameInfo, theRobotPose, theTeamMateData, theFieldDimensions, theRobotsModel, theOwnTeamInfo, theTorsoMatrix, theRobotModel, theRobotDimensions));
  symbols.push_back(new BH2011BIKESymbols(behaviorControlOutput.motionRequest, theFieldDimensions, theRobotPose));
  symbols.push_back(new BH2011FallDownSymbols(theFallDownState));
  symbols.push_back(new BH2011FieldSymbols(theFieldDimensions));
  symbols.push_back(new BH2011GameSymbols(behaviorControlOutput.behaviorData, behaviorControlOutput.robotInfo, behaviorControlOutput.ownTeamInfo, behaviorControlOutput.gameInfo, theFrameInfo,theBallModel, theRobotPose));
  symbols.push_back(new BH2011GoalSymbols(theFreePartOfOpponentGoalModel, theRobotPose, theFallDownState, theGoalPercept, theGroundContactState, theGameInfo, theFrameInfo, theFieldDimensions, theDamageConfiguration));
  symbols.push_back(new BH2011HeadSymbols(behaviorControlOutput.headMotionRequest, behaviorControlOutput.headMotionProposal,theHeadJointRequest, theFieldDimensions, theRobotPose));
  symbols.push_back(new BH2011BehaviorLEDSymbols(behaviorControlOutput.behaviorLEDRequest));
  symbols.push_back(new BH2011LocatorSymbols(theRobotPose, theRobotPoseInfo, theFrameInfo, theGroundContactState, theFieldDimensions));
  symbols.push_back(new BH2011ObstacleSymbols(theFrameInfo, theObstacleModel, theArmContactModel ,theRegionPercept, theBallPercept));
  symbols.push_back(new BH2011SoccerSymbols(behaviorControlOutput.behaviorData, theGoalPercept, theRobotPose, theFrameInfo, theFieldDimensions, theBallModel));
  symbols.push_back(new BH2011TeamSymbols(theRobotInfo, theRobotPose, theBallModel, theTeamMateData, theFieldDimensions, theFrameInfo, behaviorControlOutput.behaviorData));
  symbols.push_back(new BH2011RoleSymbols(behaviorControlOutput.behaviorData, theRobotInfo, theBallModel,theTeamMateData));
  BHXabslEngineExecutor::init();
  ASSERT(pEngine);

  if(!errorHandler.errorsOccurred)
    currentAgent = pEngine->getSelectedAgentName();
}

void BH2011BehaviorControl::init()
{
  // init symbols
  for(std::list<Symbols*>::iterator i = symbols.begin(); i != symbols.end(); ++i)
    (*i)->init();
}

BH2011BehaviorControl::~BH2011BehaviorControl()
{
  for(std::list<Symbols*>::iterator i = symbols.begin(); i != symbols.end(); ++i)
    delete *i;
}

void BH2011BehaviorControl::registerSymbolsAndBasicBehaviors()
{
  for(std::list<Symbols*>::iterator i = symbols.begin(); i != symbols.end(); ++i)
    (*i)->registerSymbols(*pEngine);
}

void BH2011BehaviorControl::executeIfEngineCouldNotBeCreated()
{
#ifdef TARGET_ROBOT
  ASSERT(false);
#endif
}

void BH2011BehaviorControl::printGeneratedMainActionToString(char* buf) const
{
  behaviorControlOutput.motionRequest.printOut(buf);
}

bool BH2011BehaviorControl::handleMessage(InMessage& message)
{
  if(theInstance)
    return theInstance->handleXabslMessage(message);
  else
    return false;
}

void BH2011BehaviorControl::update(BehaviorControlOutput& behaviorControlOutput)
{
  this->behaviorControlOutput.ownTeamInfo = theOwnTeamInfo;
  this->behaviorControlOutput.robotInfo = theRobotInfo;
  this->behaviorControlOutput.gameInfo = theGameInfo;

  // update symbols
  for(std::list<Symbols*>::iterator i = symbols.begin(); i != symbols.end(); ++i)
    (*i)->update();

  // set agent
  if(currentAgent != theBehaviorConfiguration.agent)
  {
    currentAgent = theBehaviorConfiguration.agent;
    if(!setSelectedAgent(currentAgent.c_str()))
    {
      OUTPUT(idText, text, "BH2011BehaviorControl: Unable to enable selected agent \"" << currentAgent << "\".");
    }
  }

  // execute the engine
  pEngine->setAgentPriority(theRobotInfo.number);
  pEngine->setSynchronizationTicks(int(syncTicks));
  pEngine->prepareIncomingMessages();
  for(int i = TeamMateData::firstPlayer; i < TeamMateData::numOfPlayers; i++)
    if(i != theRobotInfo.number && theFrameInfo.getTimeSince(theTeamMateData.timeStamps[i]) < TeamDataProvider::networkTimeout
       && !theTeamMateData.isPenalized[i] && theFrameInfo.getTimeSince(theTeamMateData.timeSinceLastGroundContact[i] < 2000))
      pEngine->processIncomingMessage(theTeamMateData.behaviorData[i].xabslMessage);
  executeEngine();

  pEngine->generateOutgoingMessage(this->behaviorControlOutput.behaviorData.xabslMessage);

  behaviorControlOutput = this->behaviorControlOutput;
  behaviorControlOutput.behaviorData.teamColor = theOwnTeamInfo.teamColor == TEAM_BLUE ? BehaviorData::blue : BehaviorData::red;
  TEAM_OUTPUT_FAST(idTeamMateBehaviorData, bin, behaviorControlOutput.behaviorData);
}

void BH2011BehaviorControl::update(MotionRequest& motionRequest)
{
  motionRequest = theBehaviorControlOutput.motionRequest;
  if(Global::getReleaseOptions().motionRequest)
  {
    TEAM_OUTPUT_FAST(idMotionRequest, bin, motionRequest);
  }
}

void BH2011BehaviorControl::update(BehaviorLEDRequest& behaviorLEDRequest)
{
  behaviorLEDRequest = theBehaviorControlOutput.behaviorLEDRequest;
}

MAKE_MODULE(BH2011BehaviorControl, Behavior Control)
