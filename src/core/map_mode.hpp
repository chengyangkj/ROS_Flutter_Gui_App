// Copyright (c) 2019 Rover Robotics
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

#ifndef NAV2_MAP_SERVER__MAP_MODE_HPP_
#define NAV2_MAP_SERVER__MAP_MODE_HPP_

#include <string>
#include <vector>
namespace nav2_map_server
{
enum class MapMode
{
  Trinary,
  Scale,
  Raw,
};

const char * map_mode_to_string(MapMode map_mode);
MapMode map_mode_from_string(std::string map_mode_name);
}  // namespace nav2_map_server

#endif  // NAV2_MAP_SERVER__MAP_MODE_HPP_
