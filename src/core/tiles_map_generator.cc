#include "core/tiles_map_generator.h"

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

int TilesMapGenerator::GetMaxZoom(uint32_t width, uint32_t height) const {
  uint32_t max_dim = std::max(width, height);
  if (max_dim <= kTileSize) return 0;
  return static_cast<int>(std::ceil(std::log2(static_cast<double>(max_dim) / kTileSize)));
}

bool TilesMapGenerator::GenerateAllTilesToDir(const OccupancyGridData& map,
    const std::string& output_dir) {
  if (map.width == 0 || map.height == 0 || map.data.empty()) return false;

  int max_z = GetMaxZoom(map.width, map.height);
  uint32_t padded_w = kTileSize << max_z;
  uint32_t padded_h = kTileSize << max_z;

  std::vector<int8_t> padded(padded_w * padded_h, -1);
  for (uint32_t row = 0; row < map.height; ++row) {
    for (uint32_t col = 0; col < map.width; ++col) {
      uint32_t pad_row = map.height - 1 - row;
      padded[pad_row * padded_w + col] = map.data[row * map.width + col];
    }
  }

  namespace fs = boost::filesystem;
  fs::remove_all(output_dir);
  fs::create_directories(output_dir);

  int total = 0;
  for (int z = 0; z <= max_z; ++z) {
    int tiles_per_axis = 1 << z;
    int tile_src_size = kTileSize << (max_z - z);

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
            int8_t cell = padded[sy * padded_w + sx];
            uint8_t v = MapCellToGray(cell);
            tile.at<cv::Vec3b>(ty, tx) = cv::Vec3b(v, v, v);
          }
        }

        fs::path dir = fs::path(output_dir) / std::to_string(z) / std::to_string(x);
        fs::create_directories(dir);
        std::string path = (dir / (std::to_string(y) + ".png")).string();
        if (!cv::imwrite(path, tile)) return false;
        ++total;
      }
    }
  }
  return total > 0;
}

}  // namespace nav2_map_server
