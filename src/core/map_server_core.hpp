#pragma once

#include "core/map_io.hpp"
#include "core/topology_map.hpp"

#include <httplib.h>

#include <atomic>
#include <memory>
#include <string>
#include <thread>

namespace nav2_map_server {

class MapServerCore {
public:
  MapServerCore();
  ~MapServerCore();

  bool Initialize(int tiles_http_port = 8080,
      const std::string& tiles_output_dir = "./tiles");
  void Start();
  void Shutdown();

  LOAD_MAP_STATUS LoadMapFromYaml(const std::string& yaml_file);
  void UpdateMap(const OccupancyGridData& data);
  void UpdateTopoMap(const TopologyMap& data);
  void SetFrameId(const std::string& frame_id) { frame_id_ = frame_id; }
  void SetTopoMapFileName(const std::string& name) { topo_map_file_name_ = name; }
  std::string GetFrameId() const { return frame_id_; }
  std::string GetTopoMapFileName() const { return topo_map_file_name_; }
  bool IsMapAvailable() const { return map_available_; }
  void SetMapAvailable(bool v) { map_available_ = v; }
  void SetExtraZoomLevels(int v) { extra_zoom_levels_ = v; }
  int GetExtraZoomLevels() const { return extra_zoom_levels_; }

  const OccupancyGridData& GetMapData() const { return map_data_; }
  const TopologyMap& GetTopoMap() const { return topo_map_; }

private:
  void RegenerateTiles();
  void RunHttpServer();

  std::string frame_id_;
  std::string topo_map_file_name_;
  OccupancyGridData map_data_;
  TopologyMap topo_map_;
  bool map_available_;
  int tiles_http_port_{8080};
  std::string tiles_output_dir_;
  int extra_zoom_levels_{1};
  std::unique_ptr<std::thread> http_thread_;
  std::atomic<bool> http_running_{false};
  std::atomic<httplib::Server*> http_svr_{nullptr};
};

}  // namespace nav2_map_server
