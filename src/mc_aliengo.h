#pragma once

#include <mc_rbdyn/RobotModule.h>
#include <mc_rbdyn/RobotModuleMacros.h>
#include <mc_rbdyn_urdf/urdf.h>

#include <mc_rtc/logging.h>

#include "api.h"

namespace mc_robots
{

struct MC_ROBOTS_DLLAPI AliengoRobotModule : public mc_rbdyn::RobotModule
{
public:
  AliengoRobotModule();
};

}

extern "C"
{
  ROBOT_MODULE_API void MC_RTC_ROBOT_MODULE(std::vector<std::string> & names)
  {
    names = {"aliengo"};
  }
  ROBOT_MODULE_API void destroy(mc_rbdyn::RobotModule * ptr) { delete ptr; }
  ROBOT_MODULE_API mc_rbdyn::RobotModule * create(const std::string & n)
  {
    if(n == "aliengo")
    {
      return new mc_robots::AliengoRobotModule();
    }
    else
    {
      mc_rtc::log::error("Aliengo module Cannot create an object of type {}", n);
      return nullptr;
    }
  }
}
