#include "Utils/bush/tools/ShellTools.h"
#include "Utils/bush/tools/StringTools.h"
#include "Utils/bush/tools/Platform.h"
#include "Platform/File.h"

std::string remoteCommand(const std::string& command, const std::string ip)
{
  static std::string keyFile = (File::getBHDir() + linuxToPlatformPath("/Config/Keys/id_rsa_nao"));
  std::string ticks = "\"";
  return "ssh -i " + keyFile + " -o StrictHostKeyChecking=no root@" +
         ip + " " + ticks + command + " < /dev/null > /dev/null 2>&1 &" + ticks;
}

std::string remoteCommandForQProcess(const std::string& command, const std::string& ip)
{
  static std::string keyFile = std::string(File::getBHDir()) + linuxToPlatformPath("/Config/Keys/id_rsa_nao");
  return "ssh -i " + keyFile + " -o StrictHostKeyChecking=no root@" + ip + " " + command + "";
}

std::string scpCommand(const std::string& fromFile, const std::string& fromHost, const std::string& toDir, const std::string& toHost)
{
  static std::string keyFile = std::string(File::getBHDir()) + linuxToPlatformPath("/Config/Keys/id_rsa_nao");
  std::string from;
  if(fromHost == "")
    from = enquoteString(getLinuxPath(fromFile));
  else
    from = fromHost + ":" + enquoteString(fromFile);
  std::string to;
  if(toHost == "")
    to = enquoteString(getLinuxPath(toDir));
  else
    to = toHost + ":" + enquoteString(toDir);
  return "scp -i \"" + getLinuxPath(keyFile) + "\" -o StrictHostKeyChecking=no " + from + " " + to;
}

std::string scpCommandFromRobot(const std::string& fromDir, const std::string& ip, const std::string& toDir)
{
  return scpCommand(fromDir, "root@" + ip, toDir, "");
}

std::string scpCommandToRobot(const std::string& fromDir, const std::string& ip, const std::string& toDir)
{
  return scpCommand(fromDir, "", toDir, "root@" + ip);
}
