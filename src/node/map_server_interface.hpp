#pragma once

#include <string>

namespace nav2_map_server {

struct MapServerConfig {
  std::string yaml_filename;
  std::string pub_map_topic{"map"};
  std::string sub_map_topic{"map"};
  std::string frame_id{"map"};
  int tiles_http_port{7684};
  std::string tiles_output_dir{"./tiles"};
  double save_map_timeout{2.0};
  double free_thresh_default{0.25};
  double occupied_thresh_default{0.65};
  bool map_subscribe_transient_local{true};
};

class IMapServerNode {
public:
  virtual ~IMapServerNode() = default;
  virtual bool Init(const MapServerConfig& config) { return true; }
  virtual void Run() = 0;
  virtual void Shutdown() = 0;
};

}  // namespace nav2_map_server
