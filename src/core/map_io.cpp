/* Copyright 2019 Rover Robotics
 * Copyright 2010 Brian Gerkey
 * Copyright (c) 2008, Willow Garage, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <ORGANIZATION> nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "core/map_io.hpp"
#include "common/logger/logger.h"

#include <chrono>
#include <libgen.h>
#include <SDL2/SDL_image.h>
#include <SDL2/SDL.h>
#include <algorithm>
#include <cstdlib>
#include <cmath>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "yaml-cpp/yaml.h"

namespace nav2_map_server {

constexpr int8_t OCC_GRID_UNKNOWN = -1;
constexpr int8_t OCC_GRID_FREE = 0;
constexpr int8_t OCC_GRID_OCCUPIED = 100;

// === Map input part ===

/// Get the given subnode value.
/// The only reason this function exists is to wrap the exceptions in slightly nicer error messages,
/// including the name of the failed key
/// @throw YAML::Exception
template <typename T>
T yaml_get_value(const YAML::Node &node, const std::string &key) {
  try {
    return node[key].as<T>();
  } catch (YAML::Exception &e) {
    std::stringstream ss;
    ss << "Failed to parse YAML tag '" << key << "' for reason: " << e.msg;
    throw YAML::Exception(e.mark, ss.str());
  }
}

std::string get_home_dir() {
  if (const char *home_dir = std::getenv("HOME")) {
    return std::string{home_dir};
  }
  return std::string{};
}

std::string expand_user_home_dir_if_needed(
    std::string yaml_filename,
    std::string home_variable_value) {
  if (yaml_filename.size() < 2 || !(yaml_filename[0] == '~' && yaml_filename[1] == '/')) {
    return yaml_filename;
  }
  if (home_variable_value.empty()) {
    LOG_INFO("Map yaml file name starts with '~/' but no HOME variable set, "
        "user home dir will be not expanded");
    return yaml_filename;
  }
  const std::string prefix{home_variable_value};
  return yaml_filename.replace(0, 1, prefix);
}


LoadParameters loadMapYaml(const std::string &yaml_filename) {
  YAML::Node doc = YAML::LoadFile(expand_user_home_dir_if_needed(yaml_filename, get_home_dir()));
  LoadParameters load_parameters;

  auto image_file_name = yaml_get_value<std::string>(doc, "image");
  if (image_file_name.empty()) {
    throw YAML::Exception(doc["image"].Mark(), "The image tag was empty.");
  }
  if (image_file_name[0] != '/') {
    // dirname takes a mutable char *, so we copy into a vector
    std::vector<char> fname_copy(yaml_filename.begin(), yaml_filename.end());
    fname_copy.push_back('\0');
    image_file_name = std::string(dirname(fname_copy.data())) + '/' + image_file_name;
  }
  load_parameters.image_file_name = image_file_name;

  load_parameters.resolution = yaml_get_value<double>(doc, "resolution");
  load_parameters.origin = yaml_get_value<std::vector<double>>(doc, "origin");
  if (load_parameters.origin.size() != 3) {
    throw YAML::Exception(
        doc["origin"].Mark(), "value of the 'origin' tag should have 3 elements, not " +
                                  std::to_string(load_parameters.origin.size()));
  }

  load_parameters.free_thresh = yaml_get_value<double>(doc, "free_thresh");
  load_parameters.occupied_thresh = yaml_get_value<double>(doc, "occupied_thresh");

  auto map_mode_node = doc["mode"];
  if (!map_mode_node.IsDefined()) {
    load_parameters.mode = MapMode::Trinary;
  } else {
    load_parameters.mode = map_mode_from_string(map_mode_node.as<std::string>());
  }

  try {
    load_parameters.negate = yaml_get_value<int>(doc, "negate");
  } catch (YAML::Exception &) {
    load_parameters.negate = yaml_get_value<bool>(doc, "negate");
  }

  LOG_INFO("resolution: " << load_parameters.resolution << " origin: [" <<
      load_parameters.origin[0] << "," << load_parameters.origin[1] << "," <<
      load_parameters.origin[2] << "] free_thresh: " << load_parameters.free_thresh <<
      " occupied_thresh: " << load_parameters.occupied_thresh <<
      " mode: " << map_mode_to_string(load_parameters.mode) << " negate: " << load_parameters.negate);

  return load_parameters;
}

void loadMapFromFile(
    const LoadParameters &load_parameters,
    OccupancyGridData &map) {
  LOG_INFO("Loading image_file: " << load_parameters.image_file_name);

  SDL_Surface* img = IMG_Load(load_parameters.image_file_name.c_str());
  if (!img) {
    throw std::runtime_error(std::string("failed to open image: ") + IMG_GetError());
  }

  OccupancyGridData msg;
  msg.width = img->w;
  msg.height = img->h;
  msg.resolution = load_parameters.resolution;
  msg.origin_x = load_parameters.origin[0];
  msg.origin_y = load_parameters.origin[1];
  msg.origin_yaw = load_parameters.origin[2];
  msg.data.resize(msg.width * msg.height);

  int rowstride = img->pitch;
  int n_channels = img->format->BytesPerPixel;
  bool has_alpha = (img->format->Amask != 0);
  unsigned char* pixels = static_cast<unsigned char*>(img->pixels);

  for (int y = 0; y < static_cast<int>(msg.height); y++) {
    for (int x = 0; x < static_cast<int>(msg.width); x++) {
      unsigned char* p = pixels + y * rowstride + x * n_channels;

      std::vector<double> channels;
      for (int k = 0; k < n_channels; k++) {
        channels.push_back(static_cast<double>(p[k]));
      }
      if (load_parameters.mode == MapMode::Trinary && has_alpha && n_channels > 1) {
        channels.push_back(255.0 - static_cast<double>(p[n_channels - 1]));
      }

      unsigned char alpha_val = (n_channels > 1 && has_alpha) ? p[n_channels - 1] : 255;
      bool is_transparent = (n_channels > 1 && has_alpha) && (alpha_val < 255);

      double sum = 0;
      for (double c : channels) {
        sum += c;
      }
      double shade = sum / channels.size() / 255.0;
      double occ = (load_parameters.negate ? shade : 1.0 - shade);

      int8_t map_cell;
      switch (load_parameters.mode) {
        case MapMode::Trinary:
          if (occ > load_parameters.occupied_thresh) {
            map_cell = OCC_GRID_OCCUPIED;
          } else if (occ < load_parameters.free_thresh) {
            map_cell = OCC_GRID_FREE;
          } else {
            map_cell = OCC_GRID_UNKNOWN;
          }
          break;
        case MapMode::Scale:
          if (is_transparent) {
            map_cell = OCC_GRID_UNKNOWN;
          } else if (occ > load_parameters.occupied_thresh) {
            map_cell = OCC_GRID_OCCUPIED;
          } else if (occ < load_parameters.free_thresh) {
            map_cell = OCC_GRID_FREE;
          } else {
            double ratio = (occ - load_parameters.free_thresh) /
                (load_parameters.occupied_thresh - load_parameters.free_thresh);
            map_cell = static_cast<int8_t>(std::rint(ratio * 98.0 + 1.0));
          }
          break;
        case MapMode::Raw: {
          double occ_percent = std::round(shade * 255.0);
          if (OCC_GRID_FREE <= occ_percent && occ_percent <= OCC_GRID_OCCUPIED) {
            map_cell = static_cast<int8_t>(occ_percent);
          } else {
            map_cell = OCC_GRID_UNKNOWN;
          }
          break;
        }
        default:
          SDL_FreeSurface(img);
          throw std::runtime_error("Invalid map mode");
      }
      msg.data[msg.width * (msg.height - y - 1) + x] = map_cell;
    }
  }

  SDL_FreeSurface(img);

  LOG_INFO("Read map " << load_parameters.image_file_name
      << ": " << msg.width << " X " << msg.height << " map @ "
      << msg.resolution << " m/cell");

  map = msg;
}

void saveTopologyMapToJson(
  const nav2_map_server::TopologyMap& topo_map_msg,
  const std::string & json_file
){
  std::ofstream ofs(json_file);
  if (!ofs.is_open()) {
    LOG_ERROR("无法打开拓扑地图JSON文件: " << json_file);
    return;
  }
  nlohmann::json j=topo_map_msg;
  ofs << j.dump(2);
  ofs.close();
  LOG_INFO("保存拓扑地图JSON文件成功: " << json_file);
}

LOAD_MAP_STATUS LoadTopologyMapFromJson(const std::string & json_file, nav2_map_server::TopologyMap& topo_map_msg) {

    LOG_INFO("start to load topology map from json file: " << json_file);
    std::string file_path = expand_user_home_dir_if_needed(json_file, get_home_dir());
    std::ifstream ifs(file_path);
    if (!ifs.is_open()) {
        LOG_ERROR("无法打开拓扑地图JSON文件: " << file_path);
        return MAP_DOES_NOT_EXIST;
    }
    try {
        LOG_INFO("start to parse topology map from json file: " << json_file);
        nlohmann::json j;
        ifs >> j;
        topo_map_msg = j.get<TopologyMap>();
        
        LOG_INFO("成功加载拓扑地图: " << topo_map_msg.points.size() << " 个点, "
            << topo_map_msg.routes.size() << " 条路由");  
            
    } catch (const std::exception& e) {
        LOG_ERROR("解析拓扑地图JSON文件失败: " << e.what());
        return INVALID_MAP_DATA;
    }
    return LOAD_MAP_SUCCESS;
}


LOAD_MAP_STATUS loadMapFromYaml(
    const std::string &yaml_file,
    OccupancyGridData &map) {
  if (yaml_file.empty()) {
    LOG_ERROR("YAML file name is empty, can't load!");
    return MAP_DOES_NOT_EXIST;
  }
  LOG_INFO("Loading yaml file: " << yaml_file);
  LoadParameters load_parameters;
  try {
    load_parameters = loadMapYaml(yaml_file);
  } catch (YAML::Exception &e) {
    LOG_ERROR("Failed processing YAML file " << yaml_file << " at position (" <<
        e.mark.line << ":" << e.mark.column << ") for reason: " << e.what());
    return INVALID_MAP_METADATA;
  } catch (std::exception &e) {
    LOG_ERROR("Failed to parse map YAML loaded from file " << yaml_file << " for reason: " << e.what());
    return INVALID_MAP_METADATA;
  }
  try {
    loadMapFromFile(load_parameters, map);
  } catch (std::exception &e) {
    LOG_ERROR("Failed to load image file " << load_parameters.image_file_name << " for reason: " << e.what());
    return INVALID_MAP_DATA;
  }

  return LOAD_MAP_SUCCESS;
}

// === Map output part ===

/**
 * @brief Checks map saving parameters for consistency
 * @param save_parameters Map saving parameters.
 * NOTE: save_parameters could be updated during function execution.
 * @throw std::exception in case of inconsistent parameters
 */
void checkSaveParameters(SaveParameters &save_parameters) {
  // Checking map file name
  if (save_parameters.map_file_name == "") {
    auto now = std::chrono::system_clock::now();
    save_parameters.map_file_name = "map_" +
        std::to_string(std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch()).count());
    LOG_WARN("Map file unspecified. Map will be saved to " << save_parameters.map_file_name << " file");
  }

  // Checking thresholds
  if (save_parameters.occupied_thresh == 0.0) {
    save_parameters.occupied_thresh = 0.65;
    LOG_WARN("Occupied threshold unspecified. Setting it to default value: " << save_parameters.occupied_thresh);
  }
  if (save_parameters.free_thresh == 0.0) {
    save_parameters.free_thresh = 0.25;
    LOG_WARN("Free threshold unspecified. Setting it to default value: " << save_parameters.free_thresh);
  }
  if (1.0 < save_parameters.occupied_thresh) {
    LOG_ERROR("Threshold_occupied must be 1.0 or less");
    throw std::runtime_error("Incorrect thresholds");
  }
  if (save_parameters.free_thresh < 0.0) {
    LOG_ERROR("Free threshold must be 0.0 or greater");
    throw std::runtime_error("Incorrect thresholds");
  }
  if (save_parameters.occupied_thresh <= save_parameters.free_thresh) {
    LOG_ERROR("Threshold_free must be smaller than threshold_occupied");
    throw std::runtime_error("Incorrect thresholds");
  }

  // Checking image format
  if (save_parameters.image_format == "") {
    save_parameters.image_format = save_parameters.mode == MapMode::Scale ? "png" : "pgm";
    LOG_WARN("Image format unspecified. Setting it to: " << save_parameters.image_format);
  }

  std::transform(
      save_parameters.image_format.begin(),
      save_parameters.image_format.end(),
      save_parameters.image_format.begin(),
      [](unsigned char c) { return std::tolower(c); });

  const std::vector<std::string> BLESSED_FORMATS{"bmp", "pgm", "png"};
  if (
      std::find(BLESSED_FORMATS.begin(), BLESSED_FORMATS.end(), save_parameters.image_format) ==
      BLESSED_FORMATS.end()) {
    std::stringstream ss;
    bool first = true;
    for (auto &format_name : BLESSED_FORMATS) {
      if (!first) {
        ss << ", ";
      }
      ss << "'" << format_name << "'";
      first = false;
    }
    LOG_WARN("Requested image format '" << save_parameters.image_format << "' is not one of the recommended formats: " << ss.str());
    save_parameters.image_format = "png";
  }

  // Checking map mode
  if (
      save_parameters.mode == MapMode::Scale &&
      (save_parameters.image_format == "pgm" ||
       save_parameters.image_format == "jpg" ||
       save_parameters.image_format == "jpeg")) {
    LOG_WARN("Map mode 'scale' requires transparency, but format '" << save_parameters.image_format << "' does not support it. Consider switching to 'png'.");
  }
}

/**
 * @brief Tries to write map data into a file
 * @param map Occupancy grid data
 * @param save_parameters Map saving parameters
 * @throw std::expection in case of problem
 */
static void WritePgm(const OccupancyGridData& map,
    const SaveParameters& save_parameters, const std::string& mapdatafile) {
  int free_thresh_int = std::rint(save_parameters.free_thresh * 100.0);
  int occupied_thresh_int = std::rint(save_parameters.occupied_thresh * 100.0);

  FILE* out = fopen(mapdatafile.c_str(), "wb");
  if (!out) {
    throw std::runtime_error("Could not open file for writing: " + mapdatafile);
  }
  fprintf(out, "P5\n# CREATOR: map_server %.3f m/pix\n%d %d\n255\n",
      map.resolution, map.width, map.height);
  for (size_t y = 0; y < map.height; y++) {
    for (size_t x = 0; x < map.width; x++) {
      int8_t map_cell = map.data[map.width * (map.height - y - 1) + x];
      unsigned char val;
      if (save_parameters.mode == MapMode::Trinary) {
        if (map_cell < 0 || 100 < map_cell) {
          val = 205;
        } else if (map_cell <= free_thresh_int) {
          val = 254;
        } else if (occupied_thresh_int <= map_cell) {
          val = 0;
        } else {
          val = 205;
        }
      } else if (save_parameters.mode == MapMode::Raw) {
        val = (map_cell < 0 || 100 < map_cell) ? 255 : static_cast<unsigned char>(map_cell);
      } else {
        if (map_cell < 0 || 100 < map_cell) {
          val = 205;
        } else if (map_cell <= free_thresh_int) {
          val = 254;
        } else if (occupied_thresh_int <= map_cell) {
          val = 0;
        } else {
          val = 205;
        }
      }
      fputc(val, out);
    }
  }
  fclose(out);
}

static void WritePngOrBmp(const OccupancyGridData& map,
    const SaveParameters& save_parameters, const std::string& mapdatafile) {
  int free_thresh_int = std::rint(save_parameters.free_thresh * 100.0);
  int occupied_thresh_int = std::rint(save_parameters.occupied_thresh * 100.0);
  bool need_alpha = (save_parameters.mode == MapMode::Scale);

  Uint32 rmask = 0xff000000, gmask = 0x00ff0000, bmask = 0x0000ff00, amask = 0x000000ff;
#if SDL_BYTEORDER == SDL_BIG_ENDIAN
  rmask = 0x000000ff; gmask = 0x0000ff00; bmask = 0x00ff0000; amask = 0xff000000;
#endif

  int bpp = need_alpha ? 4 : 3;
  int pitch = map.width * bpp;
  std::vector<unsigned char> pixels(map.width * map.height * bpp);

  for (size_t y = 0; y < map.height; y++) {
    for (size_t x = 0; x < map.width; x++) {
      int8_t map_cell = map.data[map.width * (map.height - y - 1) + x];
      unsigned char r, g, b, a = 255;

      switch (save_parameters.mode) {
        case MapMode::Trinary:
          if (map_cell < 0 || 100 < map_cell) {
            r = g = b = 205;
          } else if (map_cell <= free_thresh_int) {
            r = g = b = 254;
          } else if (occupied_thresh_int <= map_cell) {
            r = g = b = 0;
          } else {
            r = g = b = 205;
          }
          break;
        case MapMode::Scale:
          if (map_cell < 0 || 100 < map_cell) {
            r = g = b = 128;
            a = 0;
          } else {
            unsigned char v = static_cast<unsigned char>((100.0 - map_cell) / 100.0 * 255.0);
            r = g = b = v;
          }
          break;
        case MapMode::Raw:
          if (map_cell < 0 || 100 < map_cell) {
            r = g = b = 255;
          } else {
            r = g = b = static_cast<unsigned char>(map_cell);
          }
          break;
        default:
          throw std::runtime_error("Invalid map mode");
      }

      size_t off = (y * map.width + x) * bpp;
      pixels[off] = r;
      pixels[off + 1] = g;
      pixels[off + 2] = b;
      if (need_alpha) {
        pixels[off + 3] = a;
      }
    }
  }

  SDL_Surface* surf = SDL_CreateRGBSurfaceFrom(
      pixels.data(), map.width, map.height, bpp * 8, pitch,
      rmask, gmask, bmask, need_alpha ? amask : 0);
  if (!surf) {
    throw std::runtime_error(std::string("SDL_CreateRGBSurfaceFrom failed: ") + SDL_GetError());
  }

  if (save_parameters.image_format == "png") {
    if (IMG_SavePNG(surf, mapdatafile.c_str()) != 0) {
      SDL_FreeSurface(surf);
      throw std::runtime_error(std::string("IMG_SavePNG failed: ") + IMG_GetError());
    }
  } else {
    if (SDL_SaveBMP(surf, mapdatafile.c_str()) != 0) {
      SDL_FreeSurface(surf);
      throw std::runtime_error(std::string("SDL_SaveBMP failed: ") + SDL_GetError());
    }
  }
  SDL_FreeSurface(surf);
}

void tryWriteMapToFile(
    const OccupancyGridData &map,
    const SaveParameters &save_parameters) {
  LOG_INFO("Received a " << map.width << " X " << map.height << " map @ " << map.resolution << " m/pix");

  std::string mapdatafile = save_parameters.map_file_name + "." + save_parameters.image_format;
  LOG_INFO("Writing map occupancy data to " << mapdatafile);
  if (save_parameters.image_format == "pgm") {
    WritePgm(map, save_parameters, mapdatafile);
  } else {
    WritePngOrBmp(map, save_parameters, mapdatafile);
  }

  std::string mapmetadatafile = save_parameters.map_file_name + ".yaml";
  {
    const int file_name_index = mapdatafile.find_last_of("/\\");
    std::string image_name = mapdatafile.substr(file_name_index + 1);

    YAML::Emitter e;
    e << YAML::Precision(3);
    e << YAML::BeginMap;
    e << YAML::Key << "image" << YAML::Value << image_name;
    e << YAML::Key << "mode" << YAML::Value << map_mode_to_string(save_parameters.mode);
    e << YAML::Key << "resolution" << YAML::Value << map.resolution;
    e << YAML::Key << "origin" << YAML::Flow << YAML::BeginSeq
        << map.origin_x << map.origin_y << map.origin_yaw << YAML::EndSeq;
    e << YAML::Key << "negate" << YAML::Value << 0;
    e << YAML::Key << "occupied_thresh" << YAML::Value << save_parameters.occupied_thresh;
    e << YAML::Key << "free_thresh" << YAML::Value << save_parameters.free_thresh;

    if (!e.good()) {
      LOG_ERROR("YAML writer failed with an error " << e.GetLastError() << ". The map metadata may be invalid.");
    }

    LOG_INFO("Writing map metadata to " << mapmetadatafile);
    std::ofstream(mapmetadatafile) << e.c_str();
  }
  LOG_INFO("Map saved");
}

bool saveMapToFile(
    const OccupancyGridData &map,
    const SaveParameters &save_parameters) {
  // Local copy of SaveParameters that might be modified by checkSaveParameters()
  SaveParameters save_parameters_loc = save_parameters;

  try {
    // Checking map parameters for consistency
    checkSaveParameters(save_parameters_loc);

    tryWriteMapToFile(map, save_parameters_loc);
  } catch (std::exception &e) {
    LOG_ERROR("Failed to write map for reason: " << e.what());
    return false;
  }
  return true;
}

}  // namespace nav2_map_server
