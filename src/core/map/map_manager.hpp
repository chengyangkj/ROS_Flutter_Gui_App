#pragma once

#include "core/map/map_io.hpp"
#include "core/map/topology_map.hpp"
#include "common/macros.h"

#include <functional>
#include <string>
#include <string_view>
#include <tuple>
#include <vector>

namespace ros_gui_backend {

using MapOperationResult = std::tuple<bool, std::string>;

class MapManager {
 public:
  using MapUpdateCallback = std::function<void()>;

 private:
  std::string ResolveCurrentMapYamlPath() const;

  std::string map_root_;
  std::string frame_id_;
  std::string topo_map_file_name_;
  OccupancyGridData map_data_;
  TopologyMap topo_map_;
  bool map_available_;
  std::string default_tiles_dir_;
  int extra_zoom_levels_{5};
  MapUpdateCallback on_map_update_cb_;

 public:
  ~MapManager();

  bool Initialize();

  void SetOnMapUpdateCallback(MapUpdateCallback cb) { on_map_update_cb_ = std::move(cb); }

  std::string GetMapRoot() const;
  std::string GetMapDir(const std::string& map_name) const;
  std::string GetTilesDir(const std::string& map_name) const;
  std::string GetDefaultTilesDir() const { return default_tiles_dir_; }
  std::string GetCurrentMapName() const;
  bool SetCurrentMapName(const std::string& map_name);

  std::vector<std::string> ListMapNames() const;
  bool ReadYamlMapMeta(const std::string& yaml_path, double& resolution, double& origin_x,
      double& origin_y, double& origin_yaw, uint32_t& width, uint32_t& height) const;
  bool TryBuildCurrentTilesMetaJson(std::string* out_json) const;

  MapOperationResult ApplyMapEditFromQuery(const std::string& session_id, const std::string& map_name,
      const std::string& topology_json, const std::string& obstacle_edits_json);
  MapOperationResult ApplyTilesExtraZoomFromJson(std::string_view body_json);
  MapOperationResult ApplyTilesExtraZoomForMapYaml(const std::string& map_name,
      std::string_view body_json);

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

  void RegenerateTiles(const std::string& output_dir);

  DEFINE_SINGLETON(MapManager)
};

}  // namespace ros_gui_backend
