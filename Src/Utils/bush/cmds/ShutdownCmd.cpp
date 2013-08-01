#include "Utils/bush/cmds/ShutdownCmd.h"
#include "Utils/bush/cmdlib/Context.h"
#include "Utils/bush/cmdlib/Commands.h"
#include "Utils/bush/cmdlib/ProcessRunner.h"
#include "Utils/bush/tools/ShellTools.h"
#include "Utils/bush/tools/StringTools.h"
#include <cstdlib>

ShutdownCmd ShutdownCmd::theShutdownCmd;

ShutdownCmd::ShutdownCmd()
{
  Commands::getInstance().addCommand(this);
}

std::string ShutdownCmd::getName() const
{
  return "shutdown";
}

std::string ShutdownCmd::getDescription() const
{
  return "bhumand stop && naoqid stop && harakiri --deep && halt";
}

bool ShutdownCmd::preExecution(Context &context, const std::vector<std::string> &params)
{
  if(!params.empty())
  {
    context.errorLine("Unrecognized parameters.");
    return false;
  }
  return true;
}

Task* ShutdownCmd::perRobotExecution(Context &context, Robot& robot)
{
  return new ShutdownTask(context, &robot);
}

ShutdownCmd::ShutdownTask::ShutdownTask(Context &context, Robot *robot)
  : RobotTask(context, robot)
{ }

bool ShutdownCmd::ShutdownTask::execute()
{
  std::string ip = robot->getBestIP();

  std::string command = remoteCommand("./bhumand stop", ip);
  ProcessRunner r(context(), fromString(command));
  r.run();
  if(r.error())
  {
    context().errorLine(robot->name + ": Failed to stop bhumand.");
    return false;
  }
  context().printLine(robot->name + ": Stopped bhumand.");
  command = remoteCommand("naoqid stop", ip);
  r = ProcessRunner(context(), fromString(command));
  r.run();
  if(r.error())
  {
    context().errorLine(robot->name + ": Failed to stop naoqid");
    return false;
  }
  context().printLine(robot->name + ": Stopped naoqi.");
  command = remoteCommand("harakiri --deep", ip);
  r = ProcessRunner(context(), fromString(command));
  r.run();
  if(r.error())
  {
    context().errorLine(robot->name + ": Ritual suicide by removal of some or all of the organs of the gastrointestinal tract FAILED!");
    return false;
  }
  context().printLine(robot->name + ": Commited ritual suicide by removal of some or all of the organs of the gastrointestinal tract.");
  command = remoteCommand("halt", ip);
  r = ProcessRunner(context(), fromString(command));
  r.run();
  if(r.error())
  {
    context().errorLine(robot->name + ": Halt failed.");
    return false;
  }
  context().printLine(robot->name + ": Shutdown finished.");
  return true;
}
