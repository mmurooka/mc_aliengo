#include "mc_aliengo.h"

#include <config_aliengo.h>

#include <RBDyn/FK.h>
#include <RBDyn/FV.h>

#include <RBDyn/parsers/urdf.h>
#include <mc_rtc/constants.h>

#include <boost/filesystem.hpp>
namespace bfs = boost::filesystem;

#include <fstream>

namespace mc_robots
{

AliengoRobotModule::AliengoRobotModule()
  : RobotModule(mc_rtc::ALIENGO_DESCRIPTION_PATH,
                "aliengo",
                std::string{mc_rtc::ALIENGO_DESCRIPTION_PATH} + std::string{"/xacro/aliengo.urdf"}
                )
{
  // Parse URDF
  init(rbd::parsers::from_urdf_file(urdf_path, false));

  /*
  // Build _convexHull
  bfs::path convexPath = bfs::path(path) / "convex/";
  for(const auto & b : mb.bodies())
  {
    bfs::path ch = convexPath / (b.name() + "_mesh-ch.txt");
    if(bfs::exists(ch))
    {
      _convexHull[b.name()] = {b.name(), ch.string()};
    }
  }
  */

  //Ref joint order
  _ref_joint_order = {"FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
		      "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
		      "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
		      "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint"
  };

  // Stance: joint name, angle in degrees
  std::map<std::string, double> standing
  {
    {"FR_hip_joint", 0.0},
    {"FR_thigh_joint", 0.67},
    {"FR_calf_joint", -1.3},
    {"FL_hip_joint", 0.0},
    {"FL_thigh_joint", 0.67},
    {"FL_calf_joint", -1.3},
    {"RR_hip_joint", 0.0},
    {"RR_thigh_joint", 0.67},
    {"RR_calf_joint", -1.3},
    {"RL_hip_joint", 0.0},
    {"RL_thigh_joint", 0.67},
    {"RL_calf_joint", -1.3},
  };

  for(const auto & j : mb.joints())
  {
    if(j.name() != "Root" && j.dof() > 0)
    {
      _stance[j.name()] = {standing.at(j.name())};
    }
  }

  _default_attitude = {1., 0., 0., 0., 0., 0., 0.60};

  _forceSensors.push_back(
      mc_rbdyn::ForceSensor("FrontLeftFootForceSensor", "FL_foot", sva::PTransformd(Eigen::Vector3d(0, 0, 0))));
  _forceSensors.push_back(
      mc_rbdyn::ForceSensor("FrontRightFootForceSensor", "FR_foot", sva::PTransformd(Eigen::Vector3d(0, 0, 0))));
  _forceSensors.push_back(
      mc_rbdyn::ForceSensor("RearLeftFootForceSensor", "RL_foot", sva::PTransformd(Eigen::Vector3d(0, 0, 0))));
  _forceSensors.push_back(
      mc_rbdyn::ForceSensor("RearRightFootForceSensor", "RR_foot", sva::PTransformd(Eigen::Vector3d(0, 0, 0))));

  _bodySensors.emplace_back("Accelerometer", "trunk", sva::PTransformd(Eigen::Vector3d(0., 0., 0.)));
  _bodySensors.emplace_back("FloatingBase", "trunk", sva::PTransformd::Identity());

  _minimalSelfCollisions = {
    {"FR_hip", "trunk", 0.02, 0.01, 0.0},
    {"FL_hip", "trunk", 0.02, 0.01, 0.0},
    {"RR_hip", "trunk", 0.02, 0.01, 0.0},
    {"RL_hip", "trunk", 0.02, 0.01, 0.0}
  };

}

}
