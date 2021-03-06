#include "Utils/bush/cmds/SSHCmd.h"
#include "Utils/bush/cmdlib/Context.h"
#include "Utils/bush/cmdlib/Commands.h"
#include "Utils/bush/cmdlib/ProcessRunner.h"
#include "Utils/bush/tools/ShellTools.h"
#include "Utils/bush/tools/StringTools.h"
#include "Utils/bush/Session.h"
#include <cstdlib>

SSHCmd SSHCmd::theSSHCmd;

SSHCmd::SSHCmd()
{
  Commands::getInstance().addCommand(this);
}

std::string SSHCmd::getName() const
{
  return "ssh";
}

std::string SSHCmd::getDescription() const
{
  return "executes and command via ssh or opens a ssh session";
}

bool SSHCmd::preExecution(Context &context, const std::vector<std::string> &params)
{
  command = "";
  for(std::vector<std::string>::const_iterator param = params.begin(); param != params.end(); ++param)
    command += " " + *param;

  return true;
}

Task* SSHCmd::perRobotExecution(Context& context, Robot &robot)
{
  return new SSHTask(context, &robot, command);
}

SSHCmd::SSHTask::SSHTask(Context &context,
                         Robot *robot,
                         const std::string &command)
  : RobotTask(context, robot),
    command(command)
{ }

bool SSHCmd::SSHTask::execute()
{
  ProcessRunner r(context(), remoteCommandForQProcess(command, robot->getBestIP()));
  r.run();

  if(r.error())
  {
    context().errorLine(robot->name + ": ssh command" + command + " failed.");
    return false;
  }
  return true;
}
