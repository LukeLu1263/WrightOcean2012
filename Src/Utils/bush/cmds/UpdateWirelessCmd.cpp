#include "Platform/File.h"
#include "Utils/bush/cmdlib/ProcessRunner.h"
#include "Utils/bush/cmds/UpdateWirelessCmd.h"
#include "Utils/bush/Session.h"
#include "Utils/bush/tools/Filesystem.h"
#include "Utils/bush/tools/StringTools.h"

static const std::string CFG_PREFIX = "wpa_supplicant.conf_";

UpdateWirelessCmd::UpdateWirelessTask::UpdateWirelessTask(Context &context,
                                                          Robot *robot,
                                                          const QString& keyFile)
  : RobotTask(context, robot),
    keyFile(keyFile)
{ }

bool UpdateWirelessCmd::UpdateWirelessTask::execute()
{
  bool status = true;
  context().printLine(robot->name);
  std::vector<std::string> configs = Filesystem::getWlanConfigs();
  for(size_t i = 0; i < configs.size(); ++i)
  {
    context().printLine("\t" + configs[i]);
    QString command = "ssh";
    QStringList args;
    args << "-q"; // This disables die man-in-the-middle warning
    args << "-i";
    args << keyFile;
    args << "-o";
    args << "StrictHostKeyChecking=no";
    args << fromString("root@" + robot->getBestIP());
    args << "cat" << "-" << ">"
         << QString("/media/userdata/system/wpa_supplicant.d/") + fromString(CFG_PREFIX + configs[i]);

    std::string filename = std::string(File::getBHDir()) + "/Install/files/" + CFG_PREFIX + configs[i];
    std::string content(Filesystem::getFileAsString(filename));
    if(content.length() < 1)
    {
      context().errorLine("Cannot read " + filename);
      continue;
    }
    QByteArray data(content.c_str());
    RemoteWriteProcessRunner r(context(), command, args, data);
    r.run();

    if(r.error())
    {
      context().errorLine("UpdateWireless of \"" + robot->name + "\" failed!");
      status = false;
    }
  }

  return status;
}

UpdateWirelessCmd::UpdateWirelessCmd()
{
  Commands::getInstance().addCommand(this);
}

std::string UpdateWirelessCmd::getName() const
{
  return "updateWireless";
}

std::string UpdateWirelessCmd::getDescription() const
{
  return "updates the wireless configurations on selected robots.";
}

bool UpdateWirelessCmd::preExecution(Context &context, const std::vector<std::string> &params)
{
  if(!params.empty())
  {
    context.errorLine("No parameters allowed.");
    return false;
  }

  keyFile = fromString(std::string(File::getBHDir()) + "/Config/Keys/id_rsa_nao");

  return true;
}

Task* UpdateWirelessCmd::perRobotExecution(Context &context, Robot &robot)
{
  return new UpdateWirelessTask(context, &robot, keyFile);
}

UpdateWirelessCmd UpdateWirelessCmd::theUpdateWirelessCmd;
