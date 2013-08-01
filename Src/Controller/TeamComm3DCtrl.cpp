/**
* @file Controller/TeamComm3DCtrl.cpp
* Declaration of a SimRobot controller that visualizes data from team comm in a SimRobot scene
* @author Colin Graf
*/

#include <QString>
#include <GL/glew.h>
#ifdef WIN32
#include <windows.h>
#endif

#include "TeamComm3DCtrl.h"
#include "Views/TeamComm3DView.h"
#include "Tools/ProcessFramework/TeamHandler.h"

TeamComm3DCtrl* TeamComm3DCtrl::controller = 0;
SimRobot::Application* TeamComm3DCtrl::application = 0;

TeamComm3DCtrl::TeamComm3DCtrl(SimRobot::Application& simRobot) : currentListener(0), currentRobotData(0)
{
  controller = this;
  application = &simRobot;

  Global::theStreamHandler = &streamHandler;
  Global::theSettings = &settings;

  port[0] = settings.teamPort;
  port[1] = settings.teamPort + 100;
  subnet[0] = subnet[1] = "255.255.255.255";
}

TeamComm3DCtrl::~TeamComm3DCtrl()
{
  qDeleteAll(views);
}

void TeamComm3DCtrl::readTeamPort()
{
  std::string name = application->getFilePath().toAscii().constData();
  int p = name.find_last_of("\\/");
  if(p >= 0)
    name = name.substr(0, p + 1);

  name += "teamPort.con";
  if(name[0] != '/' && name[0] != '\\' && (name.size() < 2 || name[1] != ':'))
    name = std::string("Scenes\\") + name;
  InBinaryFile stream(name.c_str());
  if(stream.exists())
  {
    std::string line;
    while(!stream.eof())
    {
      line.resize(0);
      while(!stream.eof())
      {
        char c[2] = " ";
        stream >> c[0];
        if(c[0] == '\n')
          break;
        else if(c[0] != '\r')
          line = line + c;
      }
      if(line.find_first_not_of(" ") != std::string::npos)
        executeConsoleCommand(line);
    }
  }
}

void TeamComm3DCtrl::executeConsoleCommand(const std::string& line)
{
  InConfigMemory stream(line.c_str(), line.size());
  std::string buffer;
  stream >> buffer;
  if(buffer == "") // comment
    return;
  else if(buffer == "tc" || buffer == "tc2")
  {
    int i = buffer == "tc" ? 0 : 1;
    stream >> port[i] >> subnet[i];
    if(subnet[i] == "")
      subnet[i] = "255.255.255.255";
  }
  else
  {
    ASSERT(false);
  }
}

bool TeamComm3DCtrl::compile()
{
  readTeamPort();

  /*
  // add team comm category in scene graph
  class Category : public SimRobot::Object
  {
  public:
    Category() : name("Views"), icon(":/Icons/folder.png") {}

  private:
    QString name;
    QIcon icon;

    virtual const QString& getDisplayName() const {return name;};
    virtual const QString& getFullName() const {return name;};
    virtual const QIcon* getIcon() const {return &icon;};
  };
  Category* category = new Category();
  views.append(category);
  application->registerObject(*this, *category, 0, SimRobot::Flag::windowless);
  */
  // team comm monitor widget
  for(int i = 0; i < 2; ++i)
  {
    TeamComm3DView* view = new TeamComm3DView(QString("port %1").arg(port[i]), i);
    views.append(view);
    application->registerObject(*this, *view, 0/*category*/, 0);
  }

  // get simulated robots
  SimRobot::Object* group = application->resolveObject("RoboCup.robots", SimRobotCore2::object);
  for(unsigned i = 0, count = application->getObjectChildCount(*group); i < count; ++i)
  {
    SimRobotCore2::Object* robot = (SimRobotCore2::Object*)application->getObjectChild(*group, i);
    QString fullName = robot->getFullName();
    QString name = fullName.mid(fullName.lastIndexOf('.') + 1);
    if(!name.startsWith("robot"))
      continue;
    TeamColor teamColor = name.endsWith("Red") ? red : name.endsWith("Blue") ? blue : numOfTeamColors;
    if(teamColor == numOfTeamColors)
      continue;
    int robotNumber = name.mid(5, 1).toInt();
    if(robotNumber < TeamMateData::firstPlayer || robotNumber >= TeamMateData::numOfPlayers)
      continue;
    PuppetData& puppetData = this->puppetData[teamColor][robotNumber];
    puppetData.teamColor = teamColor;
    puppetData.playerNumber = robotNumber;
    puppetData.robot = robot;
    const float* initialPosition = robot->getPosition();
    puppetData.initialPosition = Vector3<>(initialPosition[0] * 1000.f, initialPosition[1] * 1000.f, initialPosition[2] * 1000.f);
    puppetData.oracle.init(robot);
    robot->registerDrawing(puppetData);
  }

  // get simulated balls
  group = application->resolveObject("RoboCup.balls", SimRobotCore2::object);
  for(unsigned i = 0, count = application->getObjectChildCount(*group); i < count; ++i)
  {
    SimRobotCore2::Object* ball = (SimRobotCore2::Object*)application->getObjectChild(*group, i);
    QString fullName = ball->getFullName();
    QString name = fullName.mid(fullName.lastIndexOf('.') + 1);
    if(!name.startsWith("ball"))
      continue;
    TeamColor teamColor = name.endsWith("Red") ? red : name.endsWith("Blue") ? blue : numOfTeamColors;
    if(teamColor == numOfTeamColors)
      continue;
    int robotNumber = name.mid(4, 1).toInt();
    if(robotNumber < TeamMateData::firstPlayer || robotNumber >= TeamMateData::numOfPlayers)
      continue;
  }

  // start udp listener
  for(int i = 0; i < 2; ++i)
  {
    TeamListener& teamListener = this->teamListener[i];
    //teamListener.port = settings.teamPort + i * 100;
    teamListener.port = port[i];
    teamListener.teamHandler.start(teamListener.port, subnet[i].c_str());
    //teamListener.teamHandler.startLocal(10000 + i, 100);
  }

#ifdef WIN32
  VERIFY(timeBeginPeriod(1) == TIMERR_NOERROR); // improves precision of getCurrentTime()
#endif

  return true;
}

void TeamComm3DCtrl::update()
{
  now = SystemCall::getCurrentSystemTime();

  // poll on udp port(s)
  for(int i = 0; i < 2; ++i)
  {
    TeamListener& teamListener = this->teamListener[i];
    TeamHandler& teamHandler = teamListener.teamHandler;
    teamHandler.receive();
    if(!teamListener.in.isEmpty())
    {
      currentListener = &teamListener;
      teamListener.in.handleAllMessages(*this);
    }
    teamListener.in.clear();
    bool sendData = teamListener.ntp.doSynchronization(now, teamListener.out.out);
    if(sendData)
    {
      //teamListener.out.out.bin << releaseOptions;
      //teamListener.out.out.finishMessage(idReleaseOptions);
      teamListener.teamHandler.send();
      teamListener.out.clear();
    }
  }

  now = SystemCall::getCurrentSystemTime();

  // update robot and ball position in the simulated scene
  for(int teamColor = firstTeamColor; teamColor < numOfTeamColors; ++teamColor)
    for(int robotNumber = TeamMateData::firstPlayer; robotNumber < TeamMateData::numOfPlayers; ++robotNumber)
    {
      PuppetData& puppetData = this->puppetData[teamColor][robotNumber];
      if(puppetData.robot)
      {
        RobotData* robotData = puppetData.robotData;
        if(!robotData || now - robotData->timeStamp > 2000)
        {
          if(puppetData.online)
          {
            puppetData.oracle.moveRobot(puppetData.initialPosition, Vector3<>(0, 0, pi * (robotData ? -0.5 : 0.5)), true);
            puppetData.online = false;
            puppetData.jointData.angles[JointData::LShoulderPitch] = 0.f;
            puppetData.jointData.angles[JointData::LShoulderRoll] = 0.f;
            puppetData.jointData.angles[JointData::RShoulderPitch] = 0.f;
            puppetData.jointData.angles[JointData::RShoulderRoll] = 0.f;
            puppetData.oracle.setJointData(puppetData.jointData);
          }
        }
        else
        {
          if(!puppetData.online || robotData->jointDataTimeStamp != puppetData.lastJointDataTimeStamp)
          {
            puppetData.online = true;
            puppetData.lastJointDataTimeStamp = robotData->jointDataTimeStamp;
            if(robotData->jointDataTimeStamp && now - robotData->jointDataTimeStamp < 2000)
            {
              puppetData.oracle.setJointData(robotData->jointData);
            }
            else
            {
              puppetData.jointData.angles[JointData::LShoulderPitch] = -pi_2;
              puppetData.jointData.angles[JointData::LShoulderRoll] = 0.25f;
              puppetData.jointData.angles[JointData::RShoulderPitch] = -pi_2;
              puppetData.jointData.angles[JointData::RShoulderRoll] = 0.25f;
              puppetData.oracle.setJointData(puppetData.jointData);
            }
          }
          if(puppetData.updateTimeStamp != robotData->timeStamp || !robotData->hasGroundContact)
          {
            Pose2D robotPose = robotData->robotPose;
            if(teamColor == blue)
            {
              static const Pose2D piPose(pi);
              robotPose = piPose + robotPose;
            }
            puppetData.oracle.moveRobot(
              Vector3<>(robotPose.translation.x, robotPose.translation.y, puppetData.initialPosition.z + (robotData->hasGroundContact ? 0.f : 600.f)),
              Vector3<>(0, 0, robotPose.rotation), true);
            puppetData.updateTimeStamp = robotData->timeStamp;
          }
        }
      }
    }
}

void TeamComm3DCtrl::selectedObject(const SimRobot::Object& object)
{
  for(int teamColor = firstTeamColor; teamColor < numOfTeamColors; ++teamColor)
    for(int robotNumber = TeamMateData::firstPlayer; robotNumber < TeamMateData::numOfPlayers; ++robotNumber)
    {
      PuppetData& puppetData = this->puppetData[teamColor][robotNumber];
      puppetData.selected = puppetData.robot == &object;
    }
}

void TeamComm3DCtrl::PuppetData::draw()
{
  if(!online || !robotData)
    return;

  glScalef(0.001f, 0.001f, 0.001f);

  glPushAttrib(GL_ENABLE_BIT | GL_LINE_BIT | GL_COLOR_BUFFER_BIT | GL_LIGHTING_BIT);
  glDisable(GL_LIGHTING);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  GLUquadricObj* q = gluNewQuadric();

  // ball
  glLineWidth(4);
  glColor4ub(255, 127, 127, 127);
  glEnable(GL_BLEND);
  glBegin(GL_LINES);
  glVertex3f(0, 0, -300.f);
  Vector2<> ballPos = robotData->ballModel.estimate.position;
  glVertex3f(ballPos.x, ballPos.y, -300.f);
  glEnd();
  glDisable(GL_BLEND);
  glPushMatrix();
  glTranslatef(ballPos.x, ballPos.y, -300.f);
  glColor3f(1.f, 0.25f, 0);
  gluSphere(q, 32.5f, 16, 16);
  glPopMatrix();

  // global ball
  glLineWidth(4);
  glColor4ub(255, 127, 127, 127);
  glEnable(GL_BLEND);
  glBegin(GL_LINES);
  glVertex3f(0, 0, -300.f);
  ballPos = robotData->robotPose.invert() * robotData->combinedWorldModel.ballState.position;
  glVertex3f(ballPos.x, ballPos.y, -300.f);
  glEnd();
  glDisable(GL_BLEND);
  glPushMatrix();
  glTranslatef(ballPos.x, ballPos.y, -300.f);
  glColor3f(1.f, 1.f, 0.25f);
  gluSphere(q, 32.5f, 16, 16);
  glPopMatrix();


  /*


  */

  gluDeleteQuadric(q);
  glPopAttrib();
}

bool TeamComm3DCtrl::handleMessage(InMessage& message)
{
  switch(message.getMessageID())
  {
  case idNTPHeader:
    VERIFY(currentListener->ntp.handleMessage(message));
    message.resetReadPosition();
    {
      unsigned int ipAddress;
      message.bin >> ipAddress;
      currentRobotData = &currentListener->robotData[ipAddress];
      unsigned int sendTimeStamp;
      message.bin >> sendTimeStamp;
      message.bin >> currentRobotData->timeStamp;
      unsigned short messageSize;
      message.bin >> messageSize;
      currentRobotData->packetSizes.add(messageSize + 20 + 8); // 20 = ip header size, 8 = udp header size
      currentRobotData->packetTimeStamps.add(currentRobotData->timeStamp);
      currentRobotData->lastPacketLatency = currentListener->ntp.receiveTimeStamp - currentListener->ntp.getRemoteTimeInLocalTime(currentListener->ntp.sendTimeStamp);
      currentRobotData->ping = currentListener->ntp.getRoundTripLength();
    }
    return true;
  case idNTPIdentifier:
  case idNTPRequest:
  case idNTPResponse:
    return currentListener->ntp.handleMessage(message);

  case idRobot:
    message.bin >> currentRobotData->robotNumber;
    return true;
  case idTeamMateIsPenalized:
    message.bin >> currentRobotData->isPenalized;
    return true;
  case idTeamMateHasGroundContact:
    message.bin >> currentRobotData->hasGroundContact;
    return true;
  case idTeamMateIsUpright:
    message.bin >> currentRobotData->isUpright;
    return true;
  case idTeamMateRobotsModel:
  {
    RobotsModelCompressed robotsModelCompressed;
    message.bin >> robotsModelCompressed;
    currentRobotData->robotsModel = robotsModelCompressed.unpack();
    for(size_t i = 0; i < currentRobotData->robotsModel.robots.size(); i++)
    {
      currentRobotData->robotsModel.robots[i].timeStamp = currentListener->ntp.getRemoteTimeInLocalTime(
            currentRobotData->robotsModel.robots[i].timeStamp);
    }
    return true;
  }
  case idTeamMateRobotPose:
  {
    RobotPoseCompressed robotPoseCompressed;
    message.bin >> robotPoseCompressed;
    currentRobotData->robotPose = robotPoseCompressed.unpack();
    return true;
  }
  case idTeamMateBallModel:
  {
    BallModelCompressed ballModelCompressed;
    message.bin >> ballModelCompressed;
    currentRobotData->ballModel = ballModelCompressed.unpack();
    if(currentRobotData->ballModel.timeWhenLastSeen)
      currentRobotData->ballModel.timeWhenLastSeen = currentListener->ntp.getRemoteTimeInLocalTime(
            currentRobotData->ballModel.timeWhenLastSeen);
  }
  return true;
  case idTeamMateBallHypotheses:
  {
    BallHypotheses& ballHypotheses = currentRobotData->ballHypotheses;
    message.bin >> ballHypotheses;
    if(ballHypotheses.timeWhenDisappeared)
      ballHypotheses.timeWhenDisappeared = currentListener->ntp.getRemoteTimeInLocalTime(ballHypotheses.timeWhenDisappeared);
  }
  return true;
  case idTeamMateBehaviorData:
  {
    BehaviorData& behaviorData = currentRobotData->behaviorData;
    message.bin >> behaviorData;
    int teamColor = behaviorData.teamColor == BehaviorData::red ? red : behaviorData.teamColor == BehaviorData::blue ? blue : numOfTeamColors;
    int& robotNumber = currentRobotData->robotNumber;
    if(robotNumber >= TeamMateData::firstPlayer && robotNumber < TeamMateData::numOfPlayers &&
       teamColor >= firstTeamColor && teamColor < numOfTeamColors)
    {
      if(currentRobotData->puppetData)
        currentRobotData->puppetData->robotData = 0;
      puppetData[teamColor][robotNumber].robotData = currentRobotData;
      currentRobotData->puppetData = &puppetData[teamColor][robotNumber];
    }
  }
  return true;
  case idTeamMateBallAfterKickPose:
  {
    BallAfterKickPose& ballAfterKickPose = currentRobotData->ballAfterKickPose;
    message.bin >> ballAfterKickPose;
    ballAfterKickPose.timeWhenLastKickWasPerformed = currentListener->ntp.getRemoteTimeInLocalTime(ballAfterKickPose.timeWhenLastKickWasPerformed);
    return true;
  }
  case idTeamMatePassTarget:
    message.bin >> currentRobotData->passTarget;
    return true;
  case idRobotHealth:
    message.bin >> currentRobotData->robotHealth;
    currentRobotData->goalPercepts.add(currentRobotData->robotHealth.goalPercepts);
    currentRobotData->ballPercepts.add(currentRobotData->robotHealth.ballPercepts);
    currentRobotData->linePercepts.add(currentRobotData->robotHealth.linePercepts);
    currentRobotData->robotHealthTimeStamps.add(currentRobotData->timeStamp);
    return true;
  case idTeamMateGoalPercept:
  {
    GoalPercept& goalPercept = currentRobotData->goalPercept;
    message.bin >> goalPercept;
    if(goalPercept.timeWhenOppGoalLastSeen)
      goalPercept.timeWhenOppGoalLastSeen = currentListener->ntp.getRemoteTimeInLocalTime(goalPercept.timeWhenOppGoalLastSeen);
    if(goalPercept.timeWhenOwnGoalLastSeen)
      goalPercept.timeWhenOwnGoalLastSeen = currentListener->ntp.getRemoteTimeInLocalTime(goalPercept.timeWhenOwnGoalLastSeen);
  }
  return true;
  case idLinePercept:
    message.bin >> currentRobotData->linePercept;
    return true;
  case idMotionRequest:
    message.bin >> currentRobotData->motionRequest;;
    return true;
  case idTeamMateCombinedWorldModel:
    message.bin >> currentRobotData->combinedWorldModel;
    return true;
  case idTeamMateFreePartOfOpponentGoalModel:
    message.bin >> currentRobotData->freePartOfOpponentGoalModel;
    return true;
  case idSensorData:
    message.bin >> currentRobotData->sensorData;
    return true;
  case idJointData:
    message.bin >> currentRobotData->jointData;
    currentRobotData->jointDataTimeStamp = currentRobotData->timeStamp;
    return true;

  default:
    break;
  }
  return false;
}
