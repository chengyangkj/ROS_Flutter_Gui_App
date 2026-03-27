#include "app/application.hpp"

#include "core/map/map_manager.hpp"
#include "common/config/config.hpp"
#include "core/web_server/web_server.hpp"
#include "common/logger/logger.h"
#include "node/node_manager.hpp"

#include "yaml-cpp/yaml.h"

#include <cstring>
#include <filesystem>

#if ROS_VERSION == 1
#include <ros/ros.h>
#else
#include <rclcpp/rclcpp.hpp>
#endif

namespace ros_gui_backend {

namespace {

void YamlString(const YAML::Node& n, const char* key, std::string* dst) {
  if (n[key]) {
    *dst = n[key].as<std::string>();
  }
}

void FillSettingsFromYaml(const YAML::Node& root, AppYamlSettings* out) {
  if (root["map_manager"]) {
    const YAML::Node m = root["map_manager"];
    if (m["frame_id"]) {
      out->map_manager.frame_id = m["frame_id"].as<std::string>();
    }
  }
  if (root["ros_gui_node"]) {
    const YAML::Node n = root["ros_gui_node"];
    YamlString(n, "map_frame", &out->ros_gui_node.map_frame);
    YamlString(n, "base_link_frame", &out->ros_gui_node.base_link_frame);
    YamlString(n, "laser_topic", &out->ros_gui_node.laser_topic);
    YamlString(n, "pointcloud_topic", &out->ros_gui_node.pointcloud_topic);
    YamlString(n, "global_path_topic", &out->ros_gui_node.global_path_topic);
    YamlString(n, "local_path_topic", &out->ros_gui_node.local_path_topic);
    YamlString(n, "trace_path_topic", &out->ros_gui_node.trace_path_topic);
    YamlString(n, "odom_topic", &out->ros_gui_node.odom_topic);
    YamlString(n, "battery_topic", &out->ros_gui_node.battery_topic);
    YamlString(n, "robot_footprint_topic", &out->ros_gui_node.robot_footprint_topic);
    YamlString(n, "local_costmap_topic", &out->ros_gui_node.local_costmap_topic);
    YamlString(n, "global_costmap_topic", &out->ros_gui_node.global_costmap_topic);
    YamlString(n, "diagnostic_topic", &out->ros_gui_node.diagnostic_topic);
    YamlString(n, "topology_topic", &out->ros_gui_node.topology_topic);
    YamlString(n, "topology_json_topic", &out->ros_gui_node.topology_json_topic);
    YamlString(n, "nav_to_pose_status_topic", &out->ros_gui_node.nav_to_pose_status_topic);
    YamlString(n, "nav_through_poses_status_topic", &out->ros_gui_node.nav_through_poses_status_topic);
    YamlString(n, "reloc_topic", &out->ros_gui_node.reloc_topic);
    YamlString(n, "nav_goal_topic", &out->ros_gui_node.nav_goal_topic);
    YamlString(n, "speed_ctrl_topic", &out->ros_gui_node.speed_ctrl_topic);
    YamlString(n, "topology_publish_topic", &out->ros_gui_node.topology_publish_topic);
  }
  if (root["web_server"]) {
    const YAML::Node w = root["web_server"];
    if (w["port"]) {
      out->web_server.port = w["port"].as<int>();
    }
  }
}

}  // namespace

Application::Application() = default;

Application::~Application() {
  Stop();
}

bool Application::Initialize(int argc, char** argv) {
  argc_ = argc;
  argv_ = argv;
  config_yaml_path_ = ParseConfigYamlPathFromArgs(argc, argv);
  std::string err;
  if (!LoadSettingsFromYaml(&err)) {
    LOGGER_ERROR("Failed to load config yaml {}: {}", config_yaml_path_.empty() ? "(none)" : config_yaml_path_,
        err);
    return false;
  }
  initialized_ = true;
  return true;
}

std::string Application::ParseConfigYamlPathFromArgs(int argc, char** argv) const {
  for (int i = 1; i < argc; ++i) {
    if (std::strcmp(argv[i], "--config") == 0 && i + 1 < argc) {
      return std::string(argv[++i]);
    }
    const char prefix[] = "--config=";
    const size_t plen = sizeof(prefix) - 1;
    if (std::strncmp(argv[i], prefix, plen) == 0 && argv[i][plen] != '\0') {
      return std::string(argv[i] + plen);
    }
  }
  return std::string();
}

bool Application::LoadSettingsFromYaml(std::string* error_message) {
  settings_ = AppYamlSettings{};
  if (config_yaml_path_.empty()) {
    return true;
  }
  try {
    YAML::Node root = YAML::LoadFile(config_yaml_path_);
    if (!root.IsMap()) {
      if (error_message) {
        *error_message = "root must be a mapping";
      }
      return false;
    }
    FillSettingsFromYaml(root, &settings_);
    return true;
  } catch (const std::exception& e) {
    if (error_message) {
      *error_message = e.what();
    }
    return false;
  }
}

void Application::ShutdownRuntime() {
  WebServer::ClearSigintHook();
  if (web_server_) {
    web_server_->Shutdown();
    web_server_.reset();
  }
  NodeManager::Instance()->Reset();
  if (ros_runtime_inited_) {
#if ROS_VERSION == 1
    ros::shutdown();
#elif ROS_VERSION == 2
    rclcpp::shutdown();
#endif
  }
  ros_runtime_inited_ = false;
}

void Application::Stop() {
  if (stopped_) {
    return;
  }
  stopped_ = true;
  ShutdownRuntime();
}

int Application::Start() {
  if (!initialized_) {
    LOGGER_ERROR("Application not initialized");
    return -1;
  }
  stopped_ = false;

#if ROS_VERSION == 1
  ros::init(argc_, argv_, "ros_gui_app_backend");
#else
  rclcpp::init(argc_, argv_);
#endif
  ros_runtime_inited_ = true;

  MapManager* mm = MapManager::Instance();
  mm->SetFrameId(settings_.map_manager.frame_id);
  if (!mm->Initialize()) {
    LOGGER_ERROR("Failed to initialize map manager");
    Stop();
    return -1;
  }
  namespace fs = std::filesystem;
  const std::string gui_json_path = config_yaml_path_.empty()
      ? (fs::current_path() / "gui_app_settings.json").string()
      : (fs::path(config_yaml_path_).parent_path() / "gui_app_settings.json").string();
  SetGuiAppSettingsStoragePath(gui_json_path);
  if (!Config::Instance()->ResetGuiAppFromYamlAndFile(settings_.ros_gui_node)) {
    LOGGER_WARN("gui_app_settings.json invalid at {}", gui_json_path);
  }

  if (!NodeManager::Instance()->InitNode()) {
    Stop();
    return -1;
  }

#if ROS_VERSION == 1
  WebServer::SetSigintHook([]() { ros::requestShutdown(); });
#else
  WebServer::SetSigintHook([]() { rclcpp::shutdown(); });
#endif

  web_server_ = std::make_unique<WebServer>();
  if (!web_server_->Start(settings_.web_server)) {
    Stop();
    return -1;
  }

  NodeManager::Instance()->GetNode()->Run();

  ShutdownRuntime();
  stopped_ = true;
  return 0;
}

}  // namespace ros_gui_backend
