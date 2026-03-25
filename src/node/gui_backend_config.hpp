#pragma once

#include <string>

namespace ros_gui_backend {

struct GuiBackendConfig {
  std::string pub_map_topic{"/map_manager/map"};
  std::string sub_map_topic{"/map"};
  double save_map_timeout{2.0};
  double free_thresh_default{0.25};
  double occupied_thresh_default{0.65};
  bool map_subscribe_transient_local{true};
};

}  // namespace ros_gui_backend
