#pragma once

#include "core/occupancy_grid.hpp"

#include <string>

namespace nav2_map_server {

class TilesMapGenerator {
public:
  static constexpr int kTileSize = 256;
  static constexpr int kPartitionSize = 128;

  bool GenerateAllTilesToDir(const OccupancyGridData& map, const std::string& output_dir,
      int extra_zoom_levels);
  int GetMaxZoom(uint32_t width, uint32_t height, int extra_zoom_levels) const;
};

}  // namespace nav2_map_server
