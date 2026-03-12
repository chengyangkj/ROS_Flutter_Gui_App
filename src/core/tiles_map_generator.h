#pragma once

#include "core/occupancy_grid.hpp"

#include <string>

namespace nav2_map_server {

class TilesMapGenerator {
public:
  static constexpr int kTileSize = 256;

  bool GenerateAllTilesToDir(const OccupancyGridData& map, const std::string& output_dir);
  int GetMaxZoom(uint32_t width, uint32_t height) const;
};

}  // namespace nav2_map_server
