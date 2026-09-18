#include "kinova.h"

#include <mc_rbdyn/RobotLoader.h>
#include <mc_rbdyn/RobotModuleMacros.h>
#include <mc_rtc/logging.h>

#include <algorithm>
#include <cctype>
#include <map>

#ifdef WITH_MC_ROBOT_TOOLS
#  include <mc_robot_tools/ConnectableRobotModule.h>
#  include <mc_robot_tools/mc_robot_tools.h>
#endif

namespace
{

// ───── Self-collision config (always available) ─────
#ifdef WITH_MC_ROBOT_TOOLS

/** Check if a robot module is available (without loading it) */
bool isAvailable(const std::string & module_name, const std::vector<std::string> & available_robots)
{
  return std::find(available_robots.begin(), available_robots.end(), module_name) != available_robots.end();
}

static const std::vector<std::string> kinova_collision_links = {"base_link", "shoulder_link", "half_arm_1_link",
                                                                "half_arm_2_link"};

static constexpr double COL_I = 0.03;
static constexpr double COL_S = 0.015;
static constexpr double COL_D = 0.0;

void addToolCollisions(mc_rbdyn::RobotModule & module, const std::vector<std::string> & tool_links)
{
  for(const auto & kinova_link : kinova_collision_links)
  {
    for(const auto & tool_link : tool_links)
    {
      module._minimalSelfCollisions.push_back({kinova_link, tool_link, COL_I, COL_S, COL_D});
    }
  }
  module._commonSelfCollisions = module._minimalSelfCollisions;
}

// ═══════════════════════════════════════════════════════════════════════════
// All connectable-tool support is compiled only when mc_robot_tools is found.
// ═══════════════════════════════════════════════════════════════════════════

static const std::vector<std::string> camera_modules = mc_robot_tools::listRealSense();
static const std::vector<std::string> gripper_modules = []
{
  auto grippers = mc_robot_tools::listRobotiqGripper();

  auto hande = mc_robot_tools::listRobotiqHandE();
  grippers.insert(grippers.end(), hande.begin(), hande.end());

  return grippers;
}();
static const std::vector<std::string> bota_modules = mc_robot_tools::listBotaSensor();

static const std::vector<std::string> end_effectors = []
{
  std::vector<std::string> all_effectors;

  if(bota_modules.empty())
  {
    return all_effectors;
  }

  auto ds4 = mc_robot_tools::listDS4();
  auto plate = mc_robot_tools::listPlate();
  auto screw = mc_robot_tools::listScrew();

  all_effectors.insert(all_effectors.end(), ds4.begin(), ds4.end());
  all_effectors.insert(all_effectors.end(), plate.begin(), plate.end());
  all_effectors.insert(all_effectors.end(), screw.begin(), screw.end());

  return all_effectors;
}();

static const std::string DEFAULT_CAMERA = !camera_modules.empty() ? camera_modules.front() : "";
static const std::string DEFAULT_GRIPPER = !gripper_modules.empty() ? gripper_modules.front() : "";
static const std::string DEFAULT_BOTA = !bota_modules.empty() ? "BFT_SENS_ECAT_M8" : "";

std::string toLower(std::string s)
{
  std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return std::tolower(c); });
  return s;
}

/** Load a tool module and require it to be a ConnectableRobotModule. */
std::shared_ptr<mc_robot_tools::ConnectableRobotModule> loadTool(const std::string & name)
{
  auto mod = mc_rbdyn::RobotLoader::get_robot_module(name);
  if(!mod)
  {
    mc_rtc::log::error("Failed to load module '{}'", name);
    return nullptr;
  }

  auto connectable = std::static_pointer_cast<mc_robot_tools::ConnectableRobotModule>(mod);
  if(!connectable)
  {
    mc_rtc::log::error("Module '{}' must inherit from mc_robot_tools::ConnectableRobotModule "
                       "to be attached to Kinova",
                       name);
    return nullptr;
  }

  return connectable;
}

/** Attach a tool to a parent module using the tool's self-described frames. */
mc_rbdyn::RobotModule attachTool(mc_rbdyn::RobotModule & parent,
                                 const std::string & parent_frame,
                                 const mc_robot_tools::ConnectableRobotModule & tool,
                                 const std::string & new_name)
{
  auto connected =
      parent.connect(tool, parent_frame, tool.baseFrame(), "",
                     mc_rbdyn::RobotModule::ConnectionParameters{}.X_other_connection(tool.defaultMountingTransform()));
  connected.name = new_name;
  addToolCollisions(connected, tool.collisionLinks());
  return connected;
}

// Resolve alias: Camera -> RealSenseD435, etc.
std::string resolveAlias(const std::string & n)
{
  std::string base = n;
  std::string suffix;

  for(const auto & s : {"FloatingBase", "Callib", "CallibFloatingBase"})
  {
    auto pos = base.find(s);
    if(pos != std::string::npos)
    {
      suffix = base.substr(pos);
      base = base.substr(0, pos);
      break;
    }
  }

  static const std::map<std::string, std::string> aliases = []()
  {
    std::map<std::string, std::string> m;
    if(!DEFAULT_CAMERA.empty()) m["KinovaCamera"] = "Kinova" + DEFAULT_CAMERA;
    if(!DEFAULT_GRIPPER.empty())
    {
      m["KinovaGripper"] = "Kinova" + DEFAULT_GRIPPER;
      if(!DEFAULT_CAMERA.empty()) m["KinovaCameraGripper"] = "Kinova" + DEFAULT_CAMERA + DEFAULT_GRIPPER;
    }

    if(!DEFAULT_BOTA.empty())
    {
      m["KinovaBota"] = "Kinova" + DEFAULT_BOTA;
      for(const auto & ee : end_effectors) m["KinovaBota" + ee] = "Kinova" + DEFAULT_BOTA + ee;
    }
    return m;
  }();

  auto it = aliases.find(base);
  if(it != aliases.end())
  {
    return it->second + suffix;
  }
  return n;
}

mc_rbdyn::RobotModule * createVariants(const std::string & raw_name,
                                       const std::string & n,
                                       mc_robots::KinovaRobotModule & kinova)
{
  // Detect which tools are attached
  std::string camera_mod, gripper_mod, bota_mod, ee_mod;

  for(const auto & cam : camera_modules)
    if(n.find(cam) != std::string::npos)
    {
      camera_mod = cam;
      break;
    }
  for(const auto & grip : gripper_modules)
    if(n.find(grip) != std::string::npos)
    {
      gripper_mod = grip;
      break;
    }
  for(const auto & bota : bota_modules)
    if(n.find(bota) != std::string::npos)
    {
      bota_mod = bota;
      break;
    }
  for(const auto & ee : end_effectors)
    if(n.find(ee) != std::string::npos)
    {
      ee_mod = ee;
      break;
    }

  if(camera_mod.empty() && gripper_mod.empty() && bota_mod.empty())
  {
    return nullptr;
  }

  // Check module availability
  auto available = mc_rbdyn::RobotLoader::available_robots();
  auto checkAvail = [&](const std::string & m) -> bool
  {
    if(!m.empty() && !isAvailable(m, available))
    {
      mc_rtc::log::error("Kinova '{}' requires module '{}' which is not available", raw_name, m);
      return false;
    }
    return true;
  };

  if(!checkAvail(camera_mod) || !checkAvail(gripper_mod) || !checkAvail(bota_mod) || !checkAvail(ee_mod))
  {
    return nullptr;
  }

  // Bota variants
  if(!bota_mod.empty())
  {
    auto bota = loadTool(bota_mod);
    if(!bota) return nullptr;

    auto kinova_bota = attachTool(kinova, "tool_frame", *bota, "kinova_bota");

    if(ee_mod.empty())
    {
      return new mc_rbdyn::RobotModule(std::move(kinova_bota));
    }

    auto ee = loadTool(ee_mod);
    if(!ee) return nullptr;

    auto kinova_bota_ee = attachTool(kinova_bota, bota->wrenchFrame(), *ee, "kinova_bota_" + toLower(ee_mod));
    return new mc_rbdyn::RobotModule(std::move(kinova_bota_ee));
  }

  // Camera and/or gripper (non-bota)
  if(!camera_mod.empty())
  {
    auto camera = loadTool(camera_mod);
    if(!camera) return nullptr;

    auto kinova_camera = attachTool(kinova, "tool_frame", *camera, "kinova_camera");

    if(!gripper_mod.empty())
    {
      auto gripper = loadTool(gripper_mod);
      if(!gripper) return nullptr;

      auto kinova_cg = attachTool(kinova_camera, camera->wrenchFrame(), *gripper, "kinova_camera_gripper");
      return new mc_rbdyn::RobotModule(std::move(kinova_cg));
    }

    return new mc_rbdyn::RobotModule(std::move(kinova_camera));
  }

  if(!gripper_mod.empty())
  {
    auto gripper = loadTool(gripper_mod);
    if(!gripper) return nullptr;

    auto kinova_gripper = attachTool(kinova, "tool_frame", *gripper, "kinova_gripper");
    return new mc_rbdyn::RobotModule(std::move(kinova_gripper));
  }

  return nullptr;
}

#endif // WITH_MC_ROBOT_TOOLS

} // namespace

extern "C"
{

  ROBOT_MODULE_API void MC_RTC_ROBOT_MODULE(std::vector<std::string> & names)
  {
    names = {"Kinova", "KinovaFloatingBase"};

#ifdef WITH_MC_ROBOT_TOOLS

    // Adding Camera, Gripper, CameraGripper variants
    std::vector<std::string> generic_attachments{};
    if(!DEFAULT_CAMERA.empty())
    {
      generic_attachments.emplace_back("Camera");
    }
    if(!DEFAULT_GRIPPER.empty())
    {
      generic_attachments.emplace_back("Gripper");
      if(!DEFAULT_CAMERA.empty())
      {
        generic_attachments.emplace_back("CameraGripper");
      }
    }

    const std::vector<std::string> base_suffixes = {"", "FloatingBase"};
    auto addWithBaseSuffixes = [&](const std::string & base)
    {
      for(const auto & sfx : base_suffixes) names.push_back(base + sfx);
    };

    const std::vector<std::string> callib_suffixes = {"", "Callib", "FloatingBase", "CallibFloatingBase"};
    auto addWithCallibSuffixes = [&](const std::string & base)
    {
      for(const auto & sfx : callib_suffixes) names.push_back(base + sfx);
    };

    for(const auto & a : generic_attachments) addWithBaseSuffixes("Kinova" + a);

    for(const auto & cam : camera_modules) addWithBaseSuffixes("Kinova" + cam);
    for(const auto & grip : gripper_modules) addWithBaseSuffixes("Kinova" + grip);

    for(const auto & cam : camera_modules)
      for(const auto & grip : gripper_modules) addWithBaseSuffixes("Kinova" + cam + grip);

    // Adding bota sensor variants
    if(!DEFAULT_BOTA.empty())
    {
      std::vector<std::string> end_effector_suffixes = {""};
      for(const auto & ee : end_effectors)
      {
        if(!ee.empty())
        {
          end_effector_suffixes.push_back(ee);
        }
      }

      for(const auto & ee : end_effector_suffixes)
      {
        addWithCallibSuffixes("KinovaBota" + ee);
        for(const auto & bota : bota_modules) addWithCallibSuffixes("Kinova" + bota + ee);
      }
    }
#endif
  }

  ROBOT_MODULE_API void destroy(mc_rbdyn::RobotModule * ptr)
  {
    delete ptr;
  }

  ROBOT_MODULE_API mc_rbdyn::RobotModule * create(const std::string & raw_name)
  {
    ROBOT_MODULE_CHECK_VERSION("Kinova")

#ifdef WITH_MC_ROBOT_TOOLS
    const std::string n = resolveAlias(raw_name);
    if(n != raw_name)
    {
      mc_rtc::log::info("Kinova: '{}' resolved to '{}'", raw_name, n);
    }
#else
    const std::string n = raw_name;
#endif

    bool is_callib = n.find("Callib") != std::string::npos;
    bool is_floating_base = n.find("FloatingBase") != std::string::npos;

    if(is_callib && n.find("BFT_") == std::string::npos)
    {
      mc_rtc::log::error("Kinova: Callib mode requires a Bota variant");
      return nullptr;
    }

    auto kinova = mc_robots::KinovaRobotModule("kinova", is_callib, !is_floating_base);

    // ───── Base Kinova (always supported) ─────
    if(n == "Kinova" || n == "KinovaFloatingBase")
    {
      return new mc_rbdyn::RobotModule(std::move(kinova));
    }

#ifdef WITH_MC_ROBOT_TOOLS
    if(auto * result = createVariants(raw_name, n, kinova))
    {
      return result;
    }
#else
    mc_rtc::log::error("Kinova: variant '{}' may require mc_robot_tools, which was not found at build time. "
                       "Rebuild mc_kinova with mc_robot_tools installed to enable attached tool variants.",
                       raw_name);
#endif

    mc_rtc::log::error("Kinova module cannot create an object of type {}", raw_name);
    return nullptr;
  }
}
