#include "app/application.hpp"

#include "core/map/map_manager.hpp"
#include "core/web_server/web_server.hpp"
#include "common/logger/logger.h"

#include "yaml-cpp/yaml.h"

#include <cstring>

#if ROS_VERSION == 1
#include "node/ros1/node.hpp"
#include <ros/ros.h>
#else
#include "node/ros2/node.hpp"
#include <rclcpp/rclcpp.hpp>
#endif

namespace ros_gui_backend {

namespace {

void FillSettingsFromYaml(const YAML::Node& root, AppYamlSettings* out) {
  if (root["map_manager"]) {
    const YAML::Node m = root["map_manager"];
    if (m["frame_id"]) {
      out->map_manager.frame_id = m["frame_id"].as<std::string>();
    }
  }
  if (root["ros_gui_node"]) {
    const YAML::Node n = root["ros_gui_node"];
    if (n["pub_map_topic"]) {
      out->ros_gui_node.pub_map_topic = n["pub_map_topic"].as<std::string>();
    }
    if (n["sub_map_topic"]) {
      out->ros_gui_node.sub_map_topic = n["sub_map_topic"].as<std::string>();
    }
    if (n["save_map_timeout"]) {
      out->ros_gui_node.save_map_timeout = n["save_map_timeout"].as<double>();
    }
    if (n["free_thresh_default"]) {
      out->ros_gui_node.free_thresh_default = n["free_thresh_default"].as<double>();
    }
    if (n["occupied_thresh_default"]) {
      out->ros_gui_node.occupied_thresh_default = n["occupied_thresh_default"].as<double>();
    }
    if (n["map_subscribe_transient_local"]) {
      out->ros_gui_node.map_subscribe_transient_local =
          n["map_subscribe_transient_local"].as<bool>();
    }
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
  if (node_) {
    node_->Shutdown();
    node_.reset();
  } else if (ros_runtime_inited_) {
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

#if ROS_VERSION == 1
  node_ = std::make_shared<RosGuiNode>();
#else
  node_ = std::make_shared<RosGuiNode>(settings_.ros_gui_node);
#endif
  if (!node_->Init(settings_.ros_gui_node)) {
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

  node_->Run();

  ShutdownRuntime();
  stopped_ = true;
  return 0;
}

}  // namespace ros_gui_backend
