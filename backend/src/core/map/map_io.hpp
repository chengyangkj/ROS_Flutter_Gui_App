// Copyright (c) 2018 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef NAV2_MAP_SERVER__MAP_IO_HPP_
#define NAV2_MAP_SERVER__MAP_IO_HPP_

#include <string>
#include <vector>

#include "core/map/occupancy_grid.hpp"
#include "core/map/topology_map.hpp"
#include "core/map/map_mode.hpp"

namespace ros_gui_backend
{

struct LoadParameters
{
  std::string image_file_name;
  double resolution{0};
  std::vector<double> origin{0, 0, 0};
  double free_thresh;
  double occupied_thresh;
  MapMode mode;
  bool negate;
};

typedef enum
{
  LOAD_MAP_SUCCESS,
  MAP_DOES_NOT_EXIST,
  INVALID_MAP_METADATA,
  INVALID_MAP_DATA
} LOAD_MAP_STATUS;

LoadParameters loadMapYaml(const std::string & yaml_filename);

void loadMapFromFile(
  const LoadParameters & load_parameters,
  OccupancyGridData & map);

LOAD_MAP_STATUS loadMapFromYaml(
  const std::string & yaml_file,
  OccupancyGridData & map);

LOAD_MAP_STATUS LoadTopologyMapFromJson(
  const std::string & json_file,
  ros_gui_backend::TopologyMap& topo_map_msg
);

void saveTopologyMapToJson(
  const ros_gui_backend::TopologyMap& topo_map_msg,
  const std::string & json_file
);

struct SaveParameters
{
  std::string map_file_name{""};
  std::string image_format{""};
  double free_thresh{0.0};
  double occupied_thresh{0.0};
  MapMode mode{MapMode::Trinary};
};

bool saveMapToFile(
  const OccupancyGridData & map,
  const SaveParameters & save_parameters);

std::string expand_user_home_dir_if_needed(
  std::string yaml_filename,
  std::string home_dir);

}  // namespace ros_gui_backend

#endif  // NAV2_MAP_SERVER__MAP_IO_HPP_
