#include "../Util/b-script/src/bsNativeModule.h"

#include "bs_BH.h"
#include "bs_Tools.h"

void addGeneral(ast::ExternModuleDeclaration *module)
{
  //gameInfo
  registerEnum<GameInfoWrapper::State>(module, "GameState")
    .value("initial", GameInfoWrapper::initial, "GameInfoWrapper::initial")
    .value("ready", GameInfoWrapper::ready, "GameInfoWrapper::ready")
    .value("set", GameInfoWrapper::set, "GameInfoWrapper::set")
    .value("playing", GameInfoWrapper::playing, "GameInfoWrapper::playing")
    .value("finished", GameInfoWrapper::finished, "GameInfoWrapper::finished")
    ;
  registerEnum<GameInfoWrapper::SecondaryState>(module, "SecondaryState")
    .value("normal", GameInfoWrapper::normal, "GameInfoWrapper::normal")
    .value("penaltyshoot", GameInfoWrapper::penaltyshoot, "GameInfoWrapper::penaltyshoot")
    ;
  registerEnum<GameInfoWrapper::Team>(module, "Team")
    .value("blue", GameInfoWrapper::blue, "GameInfoWrapper::blue")
    .value("red", GameInfoWrapper::red, "GameInfoWrapper::red")
    ;

  registerClass<GameInfoWrapper>(module, "GameInfo")
    .var("state", &GameInfoWrapper::state)
    .var("firstHalf", &GameInfoWrapper::firstHalf)
    .var("kickOffTeam", &GameInfoWrapper::kickOffTeam)
    .var("secsRemaining", &GameInfoWrapper::secsRemaining)
    .var("dropInTeam", &GameInfoWrapper::dropInTeam)
    .var("dropInTime", &GameInfoWrapper::dropInTime)
    .var("secondaryState", &GameInfoWrapper::secondaryState)
    .var("timeSinceLastPackageReceived", &GameInfoWrapper::timeSinceLastPackageReceived)
    ;

  //RobotInfo
  registerEnum<RobotInfoWrapper::Penalty>(module, "Penalty")
    .value("none", RobotInfoWrapper::none, "RobotInfoWrapper::none")
    .value("manual", RobotInfoWrapper::manual, "RobotInfoWrapper::manual")
    ;

  registerClass<RobotInfoWrapper>(module, "RobotInfo")
    .var("penalty", &RobotInfoWrapper::penalty)
    .var("number", &RobotInfoWrapper::number)
    .var("role", &RobotInfoWrapper::role)
    ;

  //TeamInfo
  registerClass<TeamInfoWrapper>(module, "TeamInfo")
    .var("number", &TeamInfoWrapper::number)
    .var("color", &TeamInfoWrapper::color)
    .var("ownScore", &TeamInfoWrapper::ownScore)
    .var("oppScore", &TeamInfoWrapper::oppScore)
    ;
}

void addOutput(ast::ExternModuleDeclaration *module)
{
  //HeadMotionRequest
  registerEnum<HeadMotionRequest::Mode>(module, "HeadMotionMode")
    .value("panTilt", HeadMotionRequest::panTiltMode, "HeadMotionRequest::panTiltMode")
    .value("target", HeadMotionRequest::targetMode, "HeadMotionRequest::targetMode")
    .value("targetOnGround", HeadMotionRequest::targetOnGroundMode, "HeadMotionRequest::targetOnGroundMode")
    ;

  registerEnum<HeadMotionRequest::CameraControlMode>(module, "CameraControlMode")
    .value("auto", HeadMotionRequest::autoCamera, "HeadMotionRequest::autoCamera")
    .value("lower", HeadMotionRequest::lowerCamera, "HeadMotionRequest::lowerCamera")
    .value("upper", HeadMotionRequest::upperCamera, "HeadMotionRequest::upperCamera")
    ;

  registerClass<HeadMotionRequest>(module, "HeadMotionRequest")
    .var("target", &HeadMotionRequest::target)
    .var("pan", &HeadMotionRequest::pan)
    .var("tilt", &HeadMotionRequest::tilt)
    .var("speed", &HeadMotionRequest::speed)
    .var("mode", &HeadMotionRequest::mode)
    .var("cameraControlMode", &HeadMotionRequest::cameraControlMode)
    ;

  //MotionRequest
  registerEnum<MotionRequest::Motion>(module, "Motion")
    .value("walk", MotionRequest::walk, "MotionRequest::walk")
    .value("bike", MotionRequest::bike, "MotionRequest::bike")
    .value("specialAction", MotionRequest::specialAction, "MotionRequest::specialAction")
    .value("stand", MotionRequest::stand, "MotionRequest::stand")
    ;

  registerEnum<WalkRequest::Mode>(module, "WalkMode")
    .value("speed", WalkRequest::speedMode, "WalkRequest::speedMode")
    .value("percentage", WalkRequest::percentageSpeedMode, "WalkRequest::percentageSpeedMode")
    .value("target", WalkRequest::targetMode, "WalkRequest::targetMode")
    ;

  registerEnum<WalkRequest::KickType>(module, "WalkKickType")
    .value("none", WalkRequest::none, "WalkRequest::none")
    .value("left", WalkRequest::left, "WalkRequest::left")
    .value("right", WalkRequest::right, "WalkRequest::right")
    .value("sidewardsLeft", WalkRequest::sidewardsLeft, "WalkRequest::sidewardsLeft")
    .value("sidewardsRight", WalkRequest::sidewardsRight, "WalkRequest::sidewardsRight")
    ;

  registerClass<WalkRequest>(module, "WalkRequest")
    .var("mode", &WalkRequest::mode)
    .var("speed", &WalkRequest::speed)
    .var("target", &WalkRequest::target)
    .var("kickType", &WalkRequest::kickType)
    ;

  registerEnum<SpecialActionRequest::SpecialActionID>(module, "SpecialAction")
    .value("playDead", SpecialActionRequest::playDead, "SpecialActionRequest::playDead")
    .value("standUpBack", SpecialActionRequest::standUpBackNao, "SpecialActionRequest::standUpBackNao")
    .value("standUpFront", SpecialActionRequest::standUpFrontNao, "SpecialActionRequest::standUpFrontNao")
    .value("sitDownKeeper", SpecialActionRequest::sitDownKeeper, "SpecialActionRequest::sitDownKeeper")
    .value("goUp", SpecialActionRequest::goUp, "SpecialActionRequest::goUp")
    .value("keeperJumpLeftSign", SpecialActionRequest::keeperJumpLeftSign, "SpecialActionRequest::keeperJumpLeftSign")
    ;

  registerClass<SpecialActionRequest>(module, "SpecialActionRequest")
    .var("specialAction", &SpecialActionRequest::specialAction)
    .var("mirror", &SpecialActionRequest::mirror)
    ;

  registerEnum<BikeRequest::BMotionID>(module, "Bike")
    .value("kickForward", BikeRequest::kickForward, "BikeRequest::kickForward")
    .value("none", BikeRequest::none, "BikeRequest::none")
    ;

  registerClass<BikeRequest>(module, "BikeRequest")
    .var("mirror", &BikeRequest::mirror)
    .var("dynamical", &BikeRequest::dynamical)
    .var("bMotion", &BikeRequest::bMotionType, "bMotionType")
    ;

  registerClass<MotionRequest>(module, "MotionRequest")
    .var("motion", &MotionRequest::motion)
    .var("specialAction", &MotionRequest::specialActionRequest, "specialActionRequest")
    .var("walk", &MotionRequest::walkRequest, "walkRequest")
    .var("bike", &MotionRequest::bikeRequest, "bikeRequest")
    ;

  //roles
  registerEnum<BehaviorData::Role>(module, "Role")
    .value("keeper", BehaviorData::keeper, "BehaviorData::keeper")
    .value("supporter", BehaviorData::supporter, "BehaviorData::supporter")
    .value("striker", BehaviorData::striker, "BehaviorData::striker")
    .value("defender", BehaviorData::defender, "BehaviorData::defender")
    ;

  //BehaviorLEDRequest
  registerEnum<BehaviorLEDRequest::BehaviorLED>(module, "LEDs")
    .value("leftEye", BehaviorLEDRequest::leftEye, "BehaviorLEDRequest::leftEye")
    .value("rightEye", BehaviorLEDRequest::rightEye, "BehaviorLEDRequest::rightEye")
    .value("leftEar", BehaviorLEDRequest::leftEar, "BehaviorLEDRequest::leftEar")
    .value("rightEar", BehaviorLEDRequest::rightEar, "BehaviorLEDRequest::rightEar")
    ;

  registerEnum<BehaviorLEDRequest::EyeColor>(module, "EyeColors")
    .value("default_color", BehaviorLEDRequest::default_color, "BehaviorLEDRequest::default_color")
    .value("red", BehaviorLEDRequest::red, "BehaviorLEDRequest::red")
    .value("green", BehaviorLEDRequest::green, "BehaviorLEDRequest::green")
    .value("blue", BehaviorLEDRequest::blue, "BehaviorLEDRequest::blue")
    .value("white", BehaviorLEDRequest::white, "BehaviorLEDRequest::white")
    .value("magenta", BehaviorLEDRequest::magenta, "BehaviorLEDRequest::magenta")
    .value("yellow", BehaviorLEDRequest::yellow, "BehaviorLEDRequest::yellow")
    ;

  registerEnum<LEDRequest::LEDState>(module, "LEDState")
    .value("off", LEDRequest::off, "LEDRequest::off")
    .value("on", LEDRequest::on, "LEDRequest::on")
    .value("blinking", LEDRequest::blinking, "LEDRequest::blinking")
    .value("fastBlinking", LEDRequest::fastBlinking, "LEDRequest::fastBlinking")
    .value("half", LEDRequest::half, "LEDRequest::half")
    ;

  registerClass<LEDsOut>(module, "LEDsOut")
    .funcVoid("setState", &LEDsOut::setState)
    .funcVoid("setLeftEyeColor", &LEDsOut::setLeftEyeColor)
    .funcVoid("setRightEyeColor", &LEDsOut::setRightEyeColor)
    ;

  //SoundRequest
  ast::EnumDeclaration &soundWrapper = registerEnum<SoundRequest::Sound>(module, "Sounds");
  for(int i = SoundRequest::none; i < SoundRequest::numOfSounds; i++)
  {
    SoundRequest::Sound s = (SoundRequest::Sound)i;
    soundWrapper.value(SoundRequest::getName(s), s, std::string("SoundRequest::") + SoundRequest::getName(s));
  }

  registerClass<SoundRequest>(module, "SoundRequest")
    .var("sound", &SoundRequest::sound)
    ;

  //BehaviorOutput
  registerClass<BehaviorOutput>(module, "BehaviorOutput", "BehaviorOutput")
    .var("head", &BehaviorOutput::headMotionRequest, "headMotionRequest")
    .var("motion", &BehaviorOutput::motionRequest, "motionRequest")
    .var("sound", &BehaviorOutput::soundRequest, "soundRequest")
    .var("leds", &BehaviorOutput::ledsOut, "ledsOut")
    .var("game", &BehaviorOutput::gameInfo, "gameInfo")
    .var("robot", &BehaviorOutput::robotInfo, "robotInfo")
    .var("team", &BehaviorOutput::teamInfo, "teamInfo")
    .funcVoid("walkTo", &BehaviorOutput::walkTo)
    .funcVoid("setBikeDynPoints", &BehaviorOutput::setBikeDynPoints)
    ;
}

void addInput(ast::ExternModuleDeclaration *module)
{
  //ball
  registerClass<BallState>(module, "BallState")
    .var("position", &BallState::position)
    .var("velocity", &BallState::velocity)
    ;

  registerClass<BallInfo>(module, "BallInfo")
    .var("vision", &BallInfo::vision)
    .var("model", &BallInfo::model)
    .var("end", &BallInfo::endPosition, "endPosition")
    .var("lastSeenEstimate", &BallInfo::lastSeenEstimate)
    .var("lastSeen", &BallInfo::lastSeen)
    .var("disappeared", &BallInfo::timeWhenDisappeared, "timeWhenDisappeared")
    .var("timeWhenBallReachesOwnYAxis", &BallInfo::timeWhenBallReachesOwnYAxis)
    .var("yPosWhenBallReachesOwnYAxis", &BallInfo::yPosWhenBallReachesOwnYAxis)
    .func("getKickPoseReached", &BallInfo::getKickPoseReached)
    ;

  //keys
  registerClass<KeyStatesWrapper>(module, "KeyStates")
    .var("chest", &KeyStatesWrapper::chest)
    .var("leftFoot", &KeyStatesWrapper::leftFoot)
    .var("rightFoot", &KeyStatesWrapper::rightFoot)
    ;

  //robotPose
  registerClass<RobotPose>(module, "RobotPose", "RobotPose")
    .var("translation", &RobotPose::translation)
    .var("rotation", &RobotPose::rotation)
    .func("invert", &RobotPose::invert)
    .func("_mul_", &RobotPose::operator*, "operator*")
    ;
  
  //MotionInfo
  registerClass<MotionInfo>(module, "MotionInfo", "MotionInfo")
    .var("motion", &MotionInfo::motion)
    .var("specialAction", &MotionInfo::specialActionRequest, "specialActionRequest")
    .var("walk", &MotionInfo::walkRequest, "walkRequest")
    .var("bike", &MotionInfo::bikeRequest, "bikeRequest")
    ;

  //FallDownState
  registerEnum<FallDownState::State>(module, "FallDownStateState")
    .value("undefined", FallDownState::undefined, "FallDownState::undefined")
    .value("upright", FallDownState::upright, "FallDownState::upright")
    .value("onGround", FallDownState::onGround, "FallDownState::onGround")
    .value("staggering", FallDownState::staggering, "FallDownState::staggering")
    .value("falling", FallDownState::falling, "FallDownState::falling")
    ;

  registerEnum<FallDownState::Direction>(module, "FallDownDirection")
    .value("none", FallDownState::none, "FallDownState::none")
    .value("front", FallDownState::front, "FallDownState::front")
    .value("left", FallDownState::left, "FallDownState::left")
    .value("back", FallDownState::back, "FallDownState::back")
    .value("right", FallDownState::right, "FallDownState::right")
    ;

  registerClass<FallDownState>(module, "FallDownState")
    .var("state", &FallDownState::state)
    .var("direction", &FallDownState::direction)
    ;

  //KickInfo
  registerEnum<KickInfo::KickType>(module, "KickType")
    .value("bikeForward", KickInfo::bikeForward, "KickInfo::bikeForward")
    ;

  registerClass<KickInfoWrapper>(module, "KickInfo")
    .func("getKickPose", &KickInfoWrapper::getKickPose)
    ;

  //GoalWrapper
  registerClass<GoalWrapper::GoalInfo>(module, "GoalInfo")
    .var("completeLastSeen", &GoalWrapper::GoalInfo::completeLastSeen)
    .var("anyPostLastSeen", &GoalWrapper::GoalInfo::anyPostLastSeen)
    ;

  registerClass<GoalWrapper>(module, "GoalWrapper")
    .var("own", &GoalWrapper::own)
    .var("opp", &GoalWrapper::opp)
    ;

  //BehaviorInput
  registerClass<BehaviorInput>(module, "BehaviorInput", "BehaviorInput")
    .var("time", &BehaviorInput::time)
    .var("preInitialEnabled", &BehaviorInput::preInitialEnabled)
    .var("ball", &BehaviorInput::ball)
    .var("buttons", &BehaviorInput::keyStates, "keyStates")
    .var("game", &BehaviorInput::gameInfo, "gameInfo")
    .var("robot", &BehaviorInput::robotInfo, "robotInfo")
    .var("team", &BehaviorInput::teamInfo, "teamInfo")
    .var("locator", &BehaviorInput::locator)
    .var("motionInfo", &BehaviorInput::motionInfo)
    .var("fallDownState", &BehaviorInput::fallDownState)
    .var("kick", &BehaviorInput::kickInfo, "kickInfo")
    .var("goal", &BehaviorInput::goalWrapper, "goalWrapper");
    ;
}

BS_MODULE(BH,
  BS_REQUIRES(Math);

  addGeneral(module);
  addOutput(module);
  addInput(module);
  );

