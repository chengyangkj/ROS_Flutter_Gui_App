#include "common/config/config.hpp"

#include <boost/filesystem.hpp>
#include <fstream>

namespace fs = boost::filesystem;

namespace ros_gui_backend {

namespace {

void SetIfString(const nlohmann::json& j, const char* key, std::string* dst) {
  if (!j.contains(key) || !j[key].is_string()) {
    return;
  }
  *dst = j[key].get<std::string>();
}

}  // namespace

void SetGuiAppSettingsStoragePath(std::string path) {
  Config::Instance()->SetStoragePath(std::move(path));
}

std::string ResolvedGuiAppSettingsPath() {
  return Config::Instance()->ResolvedStoragePath();
}

Config::Config() = default;

void Config::SetStoragePath(std::string path) {
  storage_path_ = std::move(path);
}

std::string Config::ResolvedStoragePath() const {
  if (!storage_path_.empty()) {
    return storage_path_;
  }
  return (fs::current_path() / "gui_app_settings.json").string();
}

bool Config::ResetGuiAppFromYamlAndFile(const NodeConfig& yaml) {
  gui_app_ = {};
  MergeNodeConfigIntoGuiSettings(yaml, &gui_app_);
  return LoadGuiAppSettingsFile(&gui_app_);
}

void MergeNodeConfigIntoGuiSettings(const NodeConfig& c, GuiAppSettings* g) {
  g->MapFrameName = c.map_frame;
  g->BaseLinkFrameName = c.base_link_frame;
  g->LaserTopic = c.laser_topic;
  g->PointCloud2Topic = c.pointcloud_topic;
  g->GlobalPathTopic = c.global_path_topic;
  g->LocalPathTopic = c.local_path_topic;
  g->TracePathTopic = c.trace_path_topic;
  g->OdomTopic = c.odom_topic;
  g->BatteryTopic = c.battery_topic;
  g->RobotFootprintTopic = c.robot_footprint_topic;
  g->LocalCostmapTopic = c.local_costmap_topic;
  g->GlobalCostmapTopic = c.global_costmap_topic;
  g->DiagnosticTopic = c.diagnostic_topic;
  g->TopologyLiveTopic = c.topology_topic;
  g->TopologyJsonTopic = c.topology_json_topic;
  g->NavToPoseStatusTopic = c.nav_to_pose_status_topic;
  g->NavThroughPosesStatusTopic = c.nav_through_poses_status_topic;
  g->RelocTopic = c.reloc_topic;
  g->NavGoalTopic = c.nav_goal_topic;
  g->SpeedCtrlTopic = c.speed_ctrl_topic;
  g->TopologyPublishTopic = c.topology_publish_topic;
}

void GuiAppSettingsToJson(const GuiAppSettings& s, nlohmann::json* out) {
  nlohmann::json j;
  j["navToPoseStatusTopic"] = s.NavToPoseStatusTopic;
  j["navThroughPosesStatusTopic"] = s.NavThroughPosesStatusTopic;
  j["laserTopic"] = s.LaserTopic;
  j["localPathTopic"] = s.LocalPathTopic;
  j["globalPathTopic"] = s.GlobalPathTopic;
  j["tracePathTopic"] = s.TracePathTopic;
  j["OdometryTopic"] = s.OdomTopic;
  j["BatteryTopic"] = s.BatteryTopic;
  j["robotFootprintTopic"] = s.RobotFootprintTopic;
  j["localCostmapTopic"] = s.LocalCostmapTopic;
  j["globalCostmapTopic"] = s.GlobalCostmapTopic;
  j["pointCloud2Topic"] = s.PointCloud2Topic;
  j["diagnosticTopic"] = s.DiagnosticTopic;
  j["topologyMapTopic"] = s.TopologyLiveTopic;
  j["topologyJsonTopic"] = s.TopologyJsonTopic;
  j["topologyPublishTopic"] = s.TopologyPublishTopic;
  j["relocTopic"] = s.RelocTopic;
  j["navGoalTopic"] = s.NavGoalTopic;
  j["SpeedCtrlTopic"] = s.SpeedCtrlTopic;
  j["mapFrameName"] = s.MapFrameName;
  j["baseLinkFrameName"] = s.BaseLinkFrameName;
  j["sshHost"] = s.SshHost;
  j["sshPort"] = s.SshPort;
  j["sshUsername"] = s.SshUsername;
  j["sshPassword"] = s.SshPassword;
  nlohmann::json qa = nlohmann::json::array();
  for (const auto& e : s.SshQuickCommands) {
    qa.push_back(
        nlohmann::json{{"name", e.name}, {"cmd", e.cmd}, {"useSudo", e.use_sudo}});
  }
  j["sshQuickCommands"] = std::move(qa);
  *out = std::move(j);
}

void GuiAppSettingsMergeJson(const nlohmann::json& j, GuiAppSettings* s) {
  SetIfString(j, "navToPoseStatusTopic", &s->NavToPoseStatusTopic);
  SetIfString(j, "navThroughPosesStatusTopic", &s->NavThroughPosesStatusTopic);
  SetIfString(j, "laserTopic", &s->LaserTopic);
  SetIfString(j, "localPathTopic", &s->LocalPathTopic);
  SetIfString(j, "globalPathTopic", &s->GlobalPathTopic);
  SetIfString(j, "tracePathTopic", &s->TracePathTopic);
  SetIfString(j, "OdometryTopic", &s->OdomTopic);
  SetIfString(j, "BatteryTopic", &s->BatteryTopic);
  SetIfString(j, "robotFootprintTopic", &s->RobotFootprintTopic);
  SetIfString(j, "localCostmapTopic", &s->LocalCostmapTopic);
  SetIfString(j, "globalCostmapTopic", &s->GlobalCostmapTopic);
  SetIfString(j, "pointCloud2Topic", &s->PointCloud2Topic);
  SetIfString(j, "diagnosticTopic", &s->DiagnosticTopic);
  SetIfString(j, "topologyMapTopic", &s->TopologyLiveTopic);
  SetIfString(j, "topologyJsonTopic", &s->TopologyJsonTopic);
  SetIfString(j, "topologyPublishTopic", &s->TopologyPublishTopic);
  SetIfString(j, "relocTopic", &s->RelocTopic);
  SetIfString(j, "navGoalTopic", &s->NavGoalTopic);
  SetIfString(j, "SpeedCtrlTopic", &s->SpeedCtrlTopic);
  SetIfString(j, "mapFrameName", &s->MapFrameName);
  SetIfString(j, "baseLinkFrameName", &s->BaseLinkFrameName);
  SetIfString(j, "sshHost", &s->SshHost);
  SetIfString(j, "sshUsername", &s->SshUsername);
  SetIfString(j, "sshPassword", &s->SshPassword);
  if (j.contains("sshPort")) {
    if (j["sshPort"].is_number_integer()) {
      s->SshPort = j["sshPort"].get<int>();
    } else if (j["sshPort"].is_number()) {
      s->SshPort = static_cast<int>(j["sshPort"].get<double>());
    }
  }
  if (j.contains("sshQuickCommands") && j["sshQuickCommands"].is_array()) {
    s->SshQuickCommands.clear();
    for (const auto& item : j["sshQuickCommands"]) {
      if (!item.is_object() || !item.contains("name") || !item.contains("cmd")) {
        continue;
      }
      if (!item["name"].is_string() || !item["cmd"].is_string()) {
        continue;
      }
      SshQuickCommandEntry ent;
      ent.name = item["name"].get<std::string>();
      ent.cmd = item["cmd"].get<std::string>();
      ent.use_sudo = false;
      if (item.contains("useSudo")) {
        if (item["useSudo"].is_boolean()) {
          ent.use_sudo = item["useSudo"].get<bool>();
        } else if (item["useSudo"].is_number()) {
          ent.use_sudo = (item["useSudo"].get<int>() != 0);
        }
      }
      s->SshQuickCommands.push_back(std::move(ent));
    }
  }
}

bool LoadGuiAppSettingsFile(GuiAppSettings* s) {
  const std::string path = Config::Instance()->ResolvedStoragePath();
  std::ifstream ifs(path);
  if (!ifs) {
    return true;
  }
  std::string raw((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());
  try {
    nlohmann::json j = nlohmann::json::parse(raw);
    GuiAppSettingsMergeJson(j, s);
    return true;
  } catch (const std::exception&) {
    return false;
  }
}

bool SaveGuiAppSettingsFile(const GuiAppSettings& s) {
  const std::string path = Config::Instance()->ResolvedStoragePath();
  nlohmann::json j;
  GuiAppSettingsToJson(s, &j);
  std::ofstream ofs(path);
  if (!ofs) {
    return false;
  }
  ofs << j.dump(2);
  return true;
}

}  // namespace ros_gui_backend
