#include "core/tiles_map_generator.h"

#include "common/logger/logger.h"

#include <opencv2/opencv.hpp>

#include <algorithm>
#include <boost/filesystem.hpp>
#include <cmath>

namespace nav2_map_server {

namespace {

constexpr uint8_t kFreeVal = 254;
constexpr uint8_t kOccVal = 0;
constexpr uint8_t kUnknownVal = 205;
constexpr int kFreeThresh = 25;
constexpr int kOccThresh = 65;

uint8_t MapCellToGray(int8_t cell) {
  if (cell < 0 || cell > 100) return kUnknownVal;
  if (cell <= kFreeThresh) return kFreeVal;
  if (cell >= kOccThresh) return kOccVal;
  return kUnknownVal;
}

}  // namespace

int TilesMapGenerator::GetMaxZoom(uint32_t width, uint32_t height, int extra_zoom_levels) const {
  uint32_t max_dim = std::max(width, height);
  if (max_dim <= kTileSize) return std::max(0, extra_zoom_levels);
  int base = static_cast<int>(std::ceil(std::log2(static_cast<double>(max_dim) / kTileSize)));
  return base + extra_zoom_levels;
}

bool TilesMapGenerator::GenerateAllTilesToDir(const OccupancyGridData& map,
    const std::string& output_dir, int extra_zoom_levels) {
  if (map.width == 0 || map.height == 0 || map.data.empty()) return false;

  int max_z = GetMaxZoom(map.width, map.height, extra_zoom_levels);
  uint32_t padded_w = kTileSize << max_z;
  uint32_t padded_h = kTileSize << max_z;

  double world_min_x = map.origin_x;
  double world_min_y = map.origin_y;
  double world_max_x = map.origin_x + map.width * map.resolution;
  double world_max_y = map.origin_y + map.height * map.resolution;

  LOG_INFO("TilesGen: map " << map.width << "x" << map.height
      << " res=" << map.resolution << " origin=(" << map.origin_x << "," << map.origin_y << ") extra_zoom_levels=" << extra_zoom_levels);
  LOG_INFO("TilesGen: world bounds x=[" << world_min_x << "," << world_max_x
      << "] y=[" << world_min_y << "," << world_max_y << "]");
  LOG_INFO("TilesGen: max_zoom=" << max_z << " padded=" << padded_w << "x" << padded_h);

  int scale = 1 << extra_zoom_levels;

  auto SamplePadded = [&](uint32_t pc, uint32_t pr) -> int8_t {
    if (pc >= padded_w || pr >= padded_h) return -1;
    uint32_t col = pc / scale;
    uint32_t row_inv = pr / scale;
    if (col >= map.width || row_inv >= map.height) return -1;
    uint32_t row = map.height - 1 - row_inv;
    return map.data[row * map.width + col];
  };

  namespace fs = boost::filesystem;
  fs::remove_all(output_dir);
  fs::create_directories(output_dir);

  int total = 0;
  for (int z = 0; z <= max_z; ++z) {
    int tiles_per_axis = 1 << z;
    int tile_src_size = kTileSize << (max_z - z);

    std::string tile_list;
    for (int x = 0; x < tiles_per_axis; ++x) {
      for (int y = 0; y < tiles_per_axis; ++y) {
        int src_x0 = x * tile_src_size;
        int src_y0 = y * tile_src_size;

        cv::Mat tile(kTileSize, kTileSize, CV_8UC3);
        for (int ty = 0; ty < kTileSize; ++ty) {
          for (int tx = 0; tx < kTileSize; ++tx) {
            int sx = src_x0 + tx * tile_src_size / kTileSize;
            int sy = src_y0 + ty * tile_src_size / kTileSize;
            sx = std::max(0, std::min(sx, static_cast<int>(padded_w) - 1));
            sy = std::max(0, std::min(sy, static_cast<int>(padded_h) - 1));
            int8_t cell = SamplePadded(static_cast<uint32_t>(sx), static_cast<uint32_t>(sy));
            uint8_t v = MapCellToGray(cell);
            tile.at<cv::Vec3b>(ty, tx) = cv::Vec3b(v, v, v);
          }
        }

        fs::path dir = fs::path(output_dir) / std::to_string(z) / std::to_string(x);
        fs::create_directories(dir);
        std::string path = (dir / (std::to_string(y) + ".png")).string();
        if (!cv::imwrite(path, tile)) return false;
        ++total;

        if (!tile_list.empty()) tile_list += " ";
        tile_list += std::to_string(z) + "/" + std::to_string(x) + "/" + std::to_string(y);
      }
    }
    LOG_INFO("TilesGen: z=" << z << " tiles_per_axis=" << tiles_per_axis
        << " x=[0," << (tiles_per_axis - 1) << "] y=[0," << (tiles_per_axis - 1) << "] "
        << tile_list);
  }
  LOG_INFO("TilesGen: total " << total << " tiles written to " << output_dir);
  return total > 0;
}

}  // namespace nav2_map_server
