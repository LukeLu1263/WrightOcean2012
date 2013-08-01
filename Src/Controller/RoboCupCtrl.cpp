/**
 * @file Controller/RoboCupCtrl.cpp
 *
 * This file implements the class RoboCupCtrl.
 *
 * @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 * @author <A href="mailto:kspiess@tzi.de">Kai Spiess</A>
 * @author Colin Graf
 */

#include <QIcon>

#include "RoboCupCtrl.h"
#include "Platform/SimRobotQt/Robot.h"
#include "RobotConsole.h"

RoboCupCtrl* RoboCupCtrl::controller = 0;
SimRobot::Application* RoboCupCtrl::application = 0;

RoboCupCtrl::RoboCupCtrl(SimRobot::Application& application) : robotName(0), simTime(false), dragTime(true), lastTime(0), putBallBack(true), ballOut(Oracle::NONE), ballOutTime(0)
{
  this->controller = this;
  this->application = &application;
  Q_INIT_RESOURCE(Controller);
}

bool RoboCupCtrl::compile()
{
  // find simulation object
  SimRobotCore2::Simulation2* simulation2 = (SimRobotCore2::Simulation2*)application->resolveObject("Simulation2", SimRobotCore2::simulation);

  // initialize simulated time and step length
  time = 10000 - SystemCall::getRealSystemTime();
  if(simulation2)
    simStepLength =  int(simulation2->getStepLength() * 1000. + 0.5);
  else
    simStepLength = 20;
  if(simStepLength > 20)
    simStepLength = 20;

  // get interfaces to simulated objects
  if(simulation2)
  {
    SimRobot::Object* group = application->resolveObject("RoboCup.robots", SimRobotCore2::object);
    for(unsigned currentRobot = 0, count = application->getObjectChildCount(*group); currentRobot < count; ++currentRobot)
    {
      SimRobot::Object* robot = (SimRobot::Object*)application->getObjectChild(*group, currentRobot);
      const QString& fullName = robot->getFullName();
      std::string robotName = fullName.toAscii().constData();
      this->robotName = robotName.c_str();
      robots.push_back(new Robot(fullName.mid(fullName.lastIndexOf('.') + 1).toAscii().constData()));
    }
    this->robotName = 0;
  }
  const SimRobot::Object* balls = (SimRobotCore2::Object*)RoboCupCtrl::application->resolveObject("RoboCup.balls", SimRobotCore2::object);
  if(balls)
    Oracle::setBall(RoboCupCtrl::application->getObjectChild(*balls, 0));

  return true;
}

RoboCupCtrl::~RoboCupCtrl()
{
  qDeleteAll(views);
  Oracle::setBall(0);
  controller = 0;
  application = 0;
}

void RoboCupCtrl::addView(SimRobot::Object* object, const SimRobot::Object* parent, int flags)
{
  views.append(object);
  application->registerObject(*this, *object, parent, flags);
}

void RoboCupCtrl::addView(SimRobot::Object* object, const QString& categoryName, int flags)
{
  SimRobot::Object* category = application->resolveObject(categoryName);
  if(!category)
  {
    int lio = categoryName.lastIndexOf('.');
    QString subParentName = categoryName.mid(0, lio);
    QString name = categoryName.mid(lio + 1);
    category = addCategory(name, subParentName);
  }
  addView(object, category, flags);
}

SimRobot::Object* RoboCupCtrl::addCategory(const QString& name, const SimRobot::Object* parent)
{
  class Category : public SimRobot::Object
  {
  public:
    Category(const QString& name, const QString& fullName, const char* icon) : name(name), fullName(fullName), icon(icon) {}

  private:
    QString name;
    QString fullName;
    QIcon icon;

    virtual const QString& getDisplayName() const {return name;};
    virtual const QString& getFullName() const {return fullName;};
    virtual const QIcon* getIcon() const {return &icon;};
  };

  SimRobot::Object* category = new Category(name, parent ? parent->getFullName() + "." + name : name, parent ? ":/Icons/folder.png" : ":/Icons/SimRobot.png");
  views.append(category);
  application->registerObject(*this, *category, parent, SimRobot::Flag::windowless);
  return category;
}

SimRobot::Object* RoboCupCtrl::addCategory(const QString& name, const QString& parentName)
{
  SimRobot::Object* parent = application->resolveObject(parentName);
  if(!parent)
  {
    int lio = parentName.lastIndexOf('.');
    QString subParentName = parentName.mid(0, lio);
    QString name = parentName.mid(lio + 1);
    parent = addCategory(name, subParentName);
  }
  return addCategory(name, parent);
}

void RoboCupCtrl::start()
{
#ifdef WIN32
  VERIFY(timeBeginPeriod(1) == TIMERR_NOERROR);
#endif
  for(std::list<Robot*>::iterator i = robots.begin(); i != robots.end(); ++i)
    (*i)->start();
}

void RoboCupCtrl::stop()
{
  for(std::list<Robot*>::iterator i = robots.begin(); i != robots.end(); ++i)
    (*i)->announceStop();
  for(std::list<Robot*>::iterator i = robots.begin(); i != robots.end(); ++i)
  {
    (*i)->stop();
    delete *i;
  }
  controller = 0;
#ifdef WIN32
  VERIFY(timeEndPeriod(1) == TIMERR_NOERROR);
#endif
}

void RoboCupCtrl::update()
{
  if(dragTime)
  {
    unsigned int t = SystemCall::getRealSystemTime();
    lastTime += simStepLength;
    if(lastTime > t) // simulation is running faster then rt
    {
      if(lastTime - t > (unsigned int)simStepLength)
        SystemCall::sleep(lastTime - t - simStepLength);
    }
    else // slower then rt
    {
      if(t - lastTime > (unsigned int)simStepLength)
        lastTime += simStepLength * ((t - lastTime) / simStepLength);
    }
  }

  if(putBallBack)
  {
    if(ballOut == Oracle::NONE)
      ballOut = Oracle::updateBall();

    if(ballOut == Oracle::GOAL_BY_BLUE)
    {
      for(std::list<Robot*>::iterator it = robots.begin(); it != robots.end(); it++)
      {
        RobotConsole* console = (*it)->getRobotProcess();
        console->handleConsole("xos game.kickoff_team game.team_color.red");
        console->handleConsole("xos game.state ready");
        console->handleConsole("xos game.drop_in_time 0");
      }
      ballOut = Oracle::WAIT_STATE_READY;
      ballOutTime = getTime() + 45000;
    }
    else if(ballOut == Oracle::GOAL_BY_RED)
    {
      for(std::list<Robot*>::iterator it = robots.begin(); it != robots.end(); it++)
      {
        RobotConsole* console = (*it)->getRobotProcess();
        console->handleConsole("xos game.kickoff_team game.team_color.blue");
        console->handleConsole("xos game.state ready");
        console->handleConsole("xos game.drop_in_time 0");
      }
      ballOut = Oracle::WAIT_STATE_READY;
      ballOutTime = getTime() + 45000;
    }
    else if(ballOut == Oracle::WAIT_STATE_READY)
    {
      bool changeState = true;
      if(getTime() < ballOutTime)
        for(std::list<Robot*>::iterator it = robots.begin(); it != robots.end() && changeState; it++)
          if(!(*it)->getRobotProcess()->isWaitingForSetState())
            changeState = false;

      if(changeState)
      {
        for(std::list<Robot*>::iterator it = robots.begin(); it != robots.end(); it++)
        {
          (*it)->getRobotProcess()->handleConsole("xos game.state set");
        }
        Oracle::moveBall(Vector3<>(0.f, 0.f, 50.f), true);
        ballOutTime =  getTime() + 5000;
        ballOut = Oracle::WAIT_STATE_SET;
      }
    }
    else if(ballOut == Oracle::WAIT_STATE_SET && getTime() > ballOutTime)
    {
      for(std::list<Robot*>::iterator it = robots.begin(); it != robots.end(); it++)
      {
        (*it)->getRobotProcess()->handleConsole("xos game.state playing");
      }
      ballOut = Oracle::NONE;
      ballOutTime = 0;
    }
    else if(ballOut == Oracle::OUT_BY_BLUE)
    {
      for(std::list<Robot*>::iterator it = robots.begin(); it != robots.end(); it++)
      {
        (*it)->getRobotProcess()->handleConsole("xos game.drop_in_team game.team_color.blue");
      }
      ballOut = Oracle::NONE;
      ballOutTime = getTime();
    }
    else if(ballOut == Oracle::OUT_BY_RED)
    {
      for(std::list<Robot*>::iterator it = robots.begin(); it != robots.end(); it++)
      {
        (*it)->getRobotProcess()->handleConsole("xos game.drop_in_team game.team_color.red");
      }
      ballOut = Oracle::NONE;
      ballOutTime = getTime();
    }
    else if(ballOut == Oracle::NONE && ballOutTime != 0)
      for(std::list<Robot*>::iterator it = robots.begin(); it != robots.end(); it++)
      {
        std::stringstream xos;
        xos << "xos game.drop_in_time " << ((getTime() - ballOutTime) / 1000.f);
        (*it)->getRobotProcess()->handleConsole(xos.str());
      }
  }

  statusText = "";
  for(std::list<Robot*>::iterator i = robots.begin(); i != robots.end(); ++i)
    (*i)->update();
  if(simTime)
    time += simStepLength;
}

std::string RoboCupCtrl::getRobotName() const
{
  unsigned threadId = Thread<ProcessBase>::getCurrentId();
  for(std::list<Robot*>::const_iterator i = robots.begin(); i != robots.end(); ++i)
    for(ProcessList::const_iterator j = (*i)->begin(); j != (*i)->end(); ++j)
      if((*j)->getId() == threadId)
        return (*i)->getName();
  if(!this->robotName)
    return "Robot1";
  std::string robotName(this->robotName);
  return robotName.substr(robotName.rfind('.') + 1);
}

std::string RoboCupCtrl::getModelName() const
{
  return "Nao";
}

unsigned RoboCupCtrl::getTime() const
{
  if(simTime)
    return unsigned(time);
  else
    return unsigned(SystemCall::getRealSystemTime() + time);
}
