#pragma once

#include "core/web_server/web_server.hpp"
#include "node/gui_backend_config.hpp"

#include <string>

namespace ros_gui_backend {

struct MapManagerYamlConfig {
  std::string frame_id{"map"};
};

struct AppYamlSettings {
  MapManagerYamlConfig map_manager;
  GuiBackendConfig ros_gui_node;
  WebServerConfig web_server;
};

}  // namespace ros_gui_backend
