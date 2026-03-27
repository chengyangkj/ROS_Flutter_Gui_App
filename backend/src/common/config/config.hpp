#pragma once

#include "common/macros.h"
#include "core/map/json.hpp"
#include "node/node_config.hpp"

#include <string>
#include <utility>
#include <vector>

namespace ros_gui_backend {

struct SshQuickCommandEntry {
  std::string name;
  std::string cmd;
  bool use_sudo = false;
};

struct GuiAppSettings {
  std::string NavToPoseStatusTopic{"/navigate_to_pose/_action/status"};
  std::string NavThroughPosesStatusTopic{"/navigate_through_poses/_action/status"};
  std::string LaserTopic{"/scan"};
  std::string LocalPathTopic{"/local_plan"};
  std::string GlobalPathTopic{"/plan"};
  std::string TracePathTopic{"/transformed_global_plan"};
  std::string OdomTopic{"/wheel/odometry"};
  std::string BatteryTopic{"/battery_status"};
  std::string RobotFootprintTopic{"/local_costmap/published_footprint"};
  std::string LocalCostmapTopic{"/local_costmap/costmap"};
  std::string GlobalCostmapTopic{"/global_costmap/costmap"};
  std::string PointCloud2Topic{"points"};
  std::string DiagnosticTopic{"/diagnostics"};
  std::string RelocTopic{"/initialpose"};
  std::string NavGoalTopic{"/goal_pose"};
  std::string SpeedCtrlTopic{"/cmd_vel"};
  std::string MapFrameName{"map"};
  std::string BaseLinkFrameName{"base_link"};
  std::string TopologyLiveTopic{"/map/topology"};
  std::string TopologyJsonTopic{};
  std::string TopologyPublishTopic{"/map/topology/update"};
  std::string SshHost;
  int SshPort = 22;
  std::string SshUsername;
  std::string SshPassword;
  std::vector<SshQuickCommandEntry> SshQuickCommands;
};

void GuiAppSettingsToJson(const GuiAppSettings& s, nlohmann::json* out);
void GuiAppSettingsMergeJson(const nlohmann::json& j, GuiAppSettings* s);
void MergeNodeConfigIntoGuiSettings(const NodeConfig& c, GuiAppSettings* g);

void SetGuiAppSettingsStoragePath(std::string path);
std::string ResolvedGuiAppSettingsPath();

bool LoadGuiAppSettingsFile(GuiAppSettings* s);
bool SaveGuiAppSettingsFile(const GuiAppSettings& s);

class Config {
 public:
  void SetStoragePath(std::string path);
  std::string ResolvedStoragePath() const;

  const GuiAppSettings& GuiApp() const { return gui_app_; }
  GuiAppSettings& MutableGuiApp() { return gui_app_; }

  bool ResetGuiAppFromYamlAndFile(const NodeConfig& yaml);

 private:
  std::string storage_path_;
  GuiAppSettings gui_app_;

  DEFINE_SINGLETON(Config)
};

}  // namespace ros_gui_backend
