#pragma once

#include "core/map_io.hpp"
#include "core/topology_map.hpp"

#include <httplib.h>

#include <atomic>
#include <functional>
#include <memory>
#include <string>
#include <thread>

namespace nav2_map_server {

class MapManager {
public:
  using MapUpdateCallback = std::function<void()>;

  MapManager();
  ~MapManager();

  bool Initialize(int tiles_http_port = 7684);
  void Start();
  void Shutdown();

  void SetOnMapUpdateCallback(MapUpdateCallback cb) { on_map_update_cb_ = std::move(cb); }

  std::string GetMapRoot() const;
  std::string GetMapDir(const std::string& map_name) const;
  std::string GetTilesDir(const std::string& map_name) const;
  std::string GetCurrentMapName() const;
  bool SetCurrentMapName(const std::string& map_name);

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
  void RegenerateTiles(const std::string& output_dir);
  void RunHttpServer();

  std::string map_root_;
  std::string frame_id_;
  std::string topo_map_file_name_;
  OccupancyGridData map_data_;
  TopologyMap topo_map_;
  bool map_available_;
  int tiles_http_port_{7684};
  std::string default_tiles_dir_;
  int extra_zoom_levels_{5};
  std::unique_ptr<std::thread> http_thread_;
  std::atomic<bool> http_running_{false};
  std::atomic<httplib::Server*> http_svr_{nullptr};
  MapUpdateCallback on_map_update_cb_;
};

}  // namespace nav2_map_server
