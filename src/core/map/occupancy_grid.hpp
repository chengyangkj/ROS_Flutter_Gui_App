#pragma once

#include <cstdint>
#include <vector>

namespace ros_gui_backend {

struct OccupancyGridData {
  uint32_t width{0};
  uint32_t height{0};
  double resolution{0};
  double origin_x{0};
  double origin_y{0};
  double origin_yaw{0};
  std::vector<int8_t> data;
};

}  // namespace ros_gui_backend
