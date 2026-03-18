#include "app/map_manager.hpp"
#include "core/tiles_map_generator.h"

#include "core/map_io.hpp"
#include "common/logger/logger.h"

#include <httplib.h>
#include "core/json.hpp"

#include <boost/filesystem.hpp>
#include <cstdlib>
#include <fstream>
#include <sstream>

#include "yaml-cpp/yaml.h"

namespace fs = boost::filesystem;

namespace nav2_map_server {

static std::string GetHomeDir() {
  if (const char* h = std::getenv("HOME")) {
    return std::string(h);
  }
  return std::string();
}

MapManager::MapManager() : map_available_(false) {
  std::string home = GetHomeDir();
  map_root_ = home.empty() ? ".maps" : home + "/.maps";
}

MapManager::~MapManager() {
  Shutdown();
}

std::string MapManager::GetMapRoot() const {
  return map_root_;
}

std::string MapManager::GetMapDir(const std::string& map_name) const {
  if (map_name.empty()) return map_root_;
  return map_root_ + "/" + map_name;
}

std::string MapManager::GetTilesDir(const std::string& map_name) const {
  return GetMapDir(map_name) + "/tiles";
}

std::string MapManager::GetCurrentMapName() const {
  std::string path = map_root_ + "/current_map";
  std::ifstream ifs(path);
  if (!ifs) return std::string("map");
  std::string name;
  if (!std::getline(ifs, name)) return std::string("map");
  size_t end = name.find_last_not_of(" \t\r\n");
  if (end != std::string::npos) name.resize(end + 1);
  else if (!name.empty()) {
    size_t start = name.find_first_not_of(" \t\r\n");
    name = (start != std::string::npos) ? name.substr(start) : "";
  }
  return name;
}

bool MapManager::SetCurrentMapName(const std::string& map_name) {
  std::string path = map_root_ + "/current_map";
  fs::create_directories(fs::path(map_root_));
  std::ofstream ofs(path);
  if (!ofs) {
    LOG_ERROR("Failed to write current_map: " << path);
    return false;
  }
  ofs << map_name;
  return true;
}



static bool ReadMapMetaFromYaml(const std::string& yaml_path, double& resolution,
    double& origin_x, double& origin_y, double& origin_yaw,
    uint32_t& width, uint32_t& height) {
  try {
    YAML::Node doc = YAML::LoadFile(yaml_path);
    resolution = doc["resolution"].as<double>();
    auto origin = doc["origin"].as<std::vector<double>>();
    if (origin.size() >= 3) {
      origin_x = origin[0];
      origin_y = origin[1];
      origin_yaw = origin[2];
    }
    if (doc["width"].IsDefined() && doc["height"].IsDefined()) {
      width = doc["width"].as<int>();
      height = doc["height"].as<int>();
      return true;
    }
    std::string img_path = doc["image"].as<std::string>();
    if (img_path[0] != '/') {
      size_t pos = yaml_path.find_last_of("/\\");
      img_path = (pos != std::string::npos ? yaml_path.substr(0, pos + 1) : "") + img_path;
    }
    std::ifstream ifs(img_path, std::ios::binary);
    if (!ifs) return false;
    std::string line;
    std::getline(ifs, line);
    if (line != "P5") return false;
    while (std::getline(ifs, line) && line[0] == '#') {}
    std::istringstream ss(line);
    int w = 0, h = 0;
    if (ss >> w >> h) {
      width = w;
      height = h;
      return true;
    }
  } catch (...) {}
  return false;
}

static std::vector<std::string> GetAllMapNames(const std::string& map_root) {
  std::vector<std::string> names;
  try {
    if (!fs::exists(map_root) || !fs::is_directory(map_root)) return names;
    for (fs::directory_iterator it(map_root); it != fs::directory_iterator(); ++it) {
      if (!fs::is_directory(it->status())) continue;
      std::string name = it->path().filename().string();
      if (name.empty() || name[0] == '.') continue;
      if (fs::exists(map_root + "/" + name + "/"+name+".yaml")) {
        names.push_back(name);
      }
    }
  } catch (...) {}
  return names;
}

bool MapManager::Initialize(int tiles_http_port) {
  tiles_http_port_ = tiles_http_port;
  default_tiles_dir_ = GetTilesDir("map");
  return true;
}

void MapManager::Start() {
  if (http_running_) return;
  http_running_ = true;
  http_thread_ = std::make_unique<std::thread>(&MapManager::RunHttpServer, this);
  LOG_INFO("Tiles HTTP server started at http://0.0.0.0:" << tiles_http_port_
      << ", URL: /tiles/{z}/{x}/{y}.png");
}

void MapManager::Shutdown() {
  http_running_ = false;
  if (auto* s = http_svr_.exchange(nullptr)) {
    s->stop();
  }
  if (http_thread_ && http_thread_->joinable()) {
    http_thread_->join();
  }
  http_thread_.reset();
}

void MapManager::RegenerateTiles(const std::string& output_dir) {
  if (!map_available_) return;
  TilesMapGenerator gen;
  if (gen.GenerateAllTilesToDir(map_data_, output_dir, extra_zoom_levels_)) {
    LOG_INFO("Tiles regenerated to " << output_dir);
  }
}

void MapManager::RunHttpServer() {
  httplib::Server svr;

  svr.set_post_routing_handler([](const httplib::Request&, httplib::Response& res) {
    res.set_header("Access-Control-Allow-Origin", "*");
  });

  svr.Options(".*", [](const httplib::Request&, httplib::Response& res) {
    res.status = 204;
    res.set_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
    res.set_header("Access-Control-Allow-Headers", "Content-Type");
    res.set_header("Access-Control-Max-Age", "86400");
  });

  auto serve_tile = [](const std::string& tiles_dir,
      const std::string& z, const std::string& x, std::string y,
      httplib::Response& res) {
    if (y.size() >= 4 && y.compare(y.size() - 4, 4, ".png") == 0) {
      y.resize(y.size() - 4);
    }
    std::string path = tiles_dir + "/" + z + "/" + x + "/" + y + ".png";
    std::ifstream ifs(path, std::ios::binary);
    if (!ifs) {
      LOG_WARN("Tile not found z=" << z << " x=" << x << " y=" << y << " path=" << path);
      res.status = 404;
      res.set_content("Not found", "text/plain");
      return;
    }
    ifs.seekg(0, std::ios::end);
    auto pos = ifs.tellg();
    ifs.seekg(0, std::ios::beg);
    if (pos < 0 || static_cast<size_t>(pos) > 512 * 1024) {
      res.status = 500;
      res.set_content("Invalid tile", "text/plain");
      return;
    }
    size_t size = static_cast<size_t>(pos);
    std::string content(size, '\0');
    if (!ifs.read(&content[0], size)) {
      res.status = 500;
      res.set_content("Read error", "text/plain");
      return;
    }
    res.set_header("Content-Type", "image/png");
    res.set_content(content, "image/png");
  };

  // GET /tiles/{map_name}/{z}/{x}/{y}.png
  // 返回指定地图的瓦片 PNG。path 中 map_name 指定地图。
  svr.Get("/tiles/:map_name/:z/:x/:y.png", [this, serve_tile](const httplib::Request& req, httplib::Response& res) {
    auto it_map = req.path_params.find("map_name");
    auto it_z = req.path_params.find("z");
    auto it_x = req.path_params.find("x");
    auto it_y = req.path_params.find("y.png");
    if (it_map == req.path_params.end() || it_z == req.path_params.end() ||
        it_x == req.path_params.end() || it_y == req.path_params.end()) {
      res.status = 400;
      res.set_content("Bad request", "text/plain");
      return;
    }
    std::string tiles_dir = GetTilesDir(it_map->second);
    serve_tile(tiles_dir, it_z->second, it_x->second, it_y->second, res);
  });

  // GET /tiles/{z}/{x}/{y}.png
  // 返回当前地图的瓦片 PNG。当前地图来自 ~/.map/current_map。
  svr.Get("/tiles/:z/:x/:y.png", [this, serve_tile](const httplib::Request& req, httplib::Response& res) {
    auto it_z = req.path_params.find("z");
    auto it_x = req.path_params.find("x");
    auto it_y = req.path_params.find("y.png");
    if (it_z == req.path_params.end() || it_x == req.path_params.end() ||
        it_y == req.path_params.end()) {
      res.status = 400;
      res.set_content("Bad request", "text/plain");
      return;
    }
    std::string current = GetCurrentMapName();
    std::string tiles_dir = current.empty() ? default_tiles_dir_ : GetTilesDir(current);
    serve_tile(tiles_dir, it_z->second, it_x->second, it_y->second, res);
  });

  // GET /updateMapEdit
  // 提交地图编辑结果。query: session_id, map_name, topology_json, obstacle_edits_json。
  // 将障碍物增量应用到当前栅格，保存到 ~/.map/{map_name}/，重生成瓦片。不切换 current_map。
  svr.Get("/updateMapEdit", [this](const httplib::Request& req, httplib::Response& res) {
    if (!map_available_) {
      res.status = 503;
      res.set_content("{\"error\":\"map not available\"}", "application/json");
      return;
    }

    auto find_param = [&req](const char* key, std::string& out) -> bool {
      if (!req.has_param(key)) return false;
      out = req.get_param_value(key);
      return true;
    };

    std::string session_id;
    std::string map_name;
    std::string topology_json;
    std::string obstacle_edits_json;

    if (!find_param("session_id", session_id) ||
        !find_param("map_name", map_name) ||
        !find_param("topology_json", topology_json) ||
        !find_param("obstacle_edits_json", obstacle_edits_json)) {
      res.status = 400;
      res.set_content("{\"error\":\"missing query params\"}", "application/json");
      return;
    }

    (void)session_id;

    std::string base_dir = GetMapDir(map_name);

    try {

      if(!fs::exists(GetTilesDir(map_name))) {
        fs::create_directories(fs::path(GetTilesDir(map_name)));
      }

      auto obstacle_edits = nlohmann::json::parse(obstacle_edits_json);
      if (!obstacle_edits.is_object()) {
        res.status = 400;
        res.set_content("{\"error\":\"obstacle_edits_json must be a JSON object\"}", "application/json");
        return;
      }

      for (auto it = obstacle_edits.begin(); it != obstacle_edits.end(); ++it) {
        const std::string key = it.key();
        if (!it.value().is_number_integer()) {
          LOG_ERROR("Invalid edit value for cellIndex: " << key << ", value type is not integer");
          continue;
        }
        const int8_t edit_value = static_cast<int8_t>(it.value().get<int>());

        int64_t cell_index = 0;
        try {
          cell_index = std::stoll(key);
        } catch (const std::exception& e) {
          LOG_ERROR("Invalid cell index: " << key << " error: " << e.what());
          continue;
        }

        if (cell_index < 0 || static_cast<size_t>(cell_index) >= map_data_.data.size()) {
          continue;
        }
        map_data_.data[static_cast<size_t>(cell_index)] = edit_value;
        LOG_INFO("Apply obstacle edit cell_index=" << cell_index
            << " edit_value=" << static_cast<int>(edit_value));
      }

      SaveParameters save_params;
      std::string map_file_base = base_dir + "/" + map_name;
      save_params.map_file_name = map_file_base;
      save_params.image_format = "pgm";
      save_params.free_thresh = 0.25;
      save_params.occupied_thresh = 0.65;
      save_params.mode = MapMode::Trinary;
      if (!saveMapToFile(map_data_, save_params)) {
        res.status = 500;
        res.set_content("{\"error\":\"failed to save map\"}", "application/json");
        return;
      }

      auto topology_obj = nlohmann::json::parse(topology_json).get<TopologyMap>();
      saveTopologyMapToJson(topology_obj, map_file_base + ".topology");

      map_available_ = true;
      RegenerateTiles(GetTilesDir(map_name));

      if (on_map_update_cb_) {
        on_map_update_cb_();
      }

      res.status = 200;
      res.set_content("{\"result\":\"ok\"}", "application/json");
    } catch (const std::exception& e) {
      res.status = 400;
      res.set_content(std::string("{\"error\":\"") + e.what() + "\"}", "application/json");
    }
  });

  // GET /tiles/
  // 接口说明页，列出各 tiles 相关接口及用法。
  svr.Get("/tiles/", [this](const httplib::Request&, httplib::Response& res) {
    res.set_content(
        "Tiles server for flutter_map.\n"
        "Tiles: /tiles/{z}/{x}/{y}.png (current) or /tiles/{map_name}/{z}/{x}/{y}.png\n"
        "Meta: /tiles/meta (current) or /tiles/{map_name}/meta\n"
        "Config: POST /tiles/config or POST /tiles/{map_name}/config\n"
        "Map list: GET /getAllMapList\n"
        "Delete map: GET /deleteMap?map_name=xxx\n"
        "Topology: GET /getTopologyMap or GET /getTopologyMap?map_name=xxx",
        "text/plain");
  });

  // GET /tiles/meta
  // 返回当前内存中地图的元信息：resolution, origin, width, height, max_zoom, extra_zoom_levels。
  svr.Get("/tiles/meta", [this](const httplib::Request&, httplib::Response& res) {
    res.set_header("Content-Type", "application/json");
    if (!map_available_) {
      res.status = 503;
      res.set_content("{\"error\":\"map not available\"}", "application/json");
      return;
    }
    TilesMapGenerator gen;
    int max_zoom = gen.GetMaxZoom(map_data_.width, map_data_.height, extra_zoom_levels_);
    nlohmann::json j;
    j["resolution"] = map_data_.resolution;
    j["origin_x"] = map_data_.origin_x;
    j["origin_y"] = map_data_.origin_y;
    j["width"] = map_data_.width;
    j["height"] = map_data_.height;
    j["max_zoom"] = max_zoom;
    j["extra_zoom_levels"] = extra_zoom_levels_;
    res.set_content(j.dump(), "application/json");
  });

  // GET /tiles/{map_name}/meta
  // 返回指定地图的元信息，从 ~/.map/{map_name}/map.yaml 读取。
  svr.Get("/tiles/:map_name/meta", [this](const httplib::Request& req, httplib::Response& res) {
    res.set_header("Content-Type", "application/json");
    auto it = req.path_params.find("map_name");
    if (it == req.path_params.end()) {
      res.status = 400;
      res.set_content("{\"error\":\"missing map_name\"}", "application/json");
      return;
    }
    std::string map_name = it->second;
    std::string yaml_path = GetMapDir(map_name) + "/map.yaml";
    if (!fs::exists(yaml_path)) {
      res.status = 404;
      res.set_content("{\"error\":\"map not found\"}", "application/json");
      return;
    }
    double resolution = 0, origin_x = 0, origin_y = 0, origin_yaw = 0;
    uint32_t width = 0, height = 0;
    if (!ReadMapMetaFromYaml(yaml_path, resolution, origin_x, origin_y, origin_yaw, width, height)) {
      res.status = 500;
      res.set_content("{\"error\":\"failed to read map meta\"}", "application/json");
      return;
    }
    TilesMapGenerator gen;
    int max_zoom = gen.GetMaxZoom(width, height, extra_zoom_levels_);
    nlohmann::json j;
    j["resolution"] = resolution;
    j["origin_x"] = origin_x;
    j["origin_y"] = origin_y;
    j["width"] = width;
    j["height"] = height;
    j["max_zoom"] = max_zoom;
    j["extra_zoom_levels"] = extra_zoom_levels_;
    res.set_content(j.dump(), "application/json");
  });

  // POST /tiles/{map_name}/config
  // 设置指定地图的 extra_zoom_levels(0-8)，加载该地图并重新生成瓦片。
  svr.Post("/tiles/:map_name/config", [this](const httplib::Request& req, httplib::Response& res) {
    res.set_header("Content-Type", "application/json");
    auto it = req.path_params.find("map_name");
    if (it == req.path_params.end()) {
      res.status = 400;
      res.set_content("{\"error\":\"missing map_name\"}", "application/json");
      return;
    }
    std::string map_name = it->second;
    std::string yaml_path = GetMapDir(map_name) + "/map.yaml";
    if (!fs::exists(yaml_path)) {
      res.status = 404;
      res.set_content("{\"error\":\"map not found\"}", "application/json");
      return;
    }
    try {
      auto j = nlohmann::json::parse(req.body);
      int v = j.value("extra_zoom_levels", extra_zoom_levels_);
      if (v < 0 || v > 8) {
        res.status = 400;
        res.set_content("{\"error\":\"extra_zoom_levels must be 0-8\"}", "application/json");
        return;
      }
      extra_zoom_levels_ = v;
      (void)LoadMapFromYaml(yaml_path);
      res.set_content("{\"extra_zoom_levels\":" + std::to_string(extra_zoom_levels_) + "}",
          "application/json");
    } catch (const std::exception& e) {
      res.status = 400;
      res.set_content("{\"error\":\"invalid json\"}", "application/json");
    }
  });

  // POST /tiles/config
  // 设置当前地图的 extra_zoom_levels(0-8)，body: {"extra_zoom_levels": N}，并重新生成瓦片。
  svr.Post("/tiles/config", [this](const httplib::Request& req, httplib::Response& res) {
    res.set_header("Content-Type", "application/json");
    if (!map_available_) {
      res.status = 503;
      res.set_content("{\"error\":\"map not available\"}", "application/json");
      return;
    }
    try {
      auto j = nlohmann::json::parse(req.body);
      int v = j.value("extra_zoom_levels", extra_zoom_levels_);
      if (v < 0 || v > 8) {
        res.status = 400;
        res.set_content("{\"error\":\"extra_zoom_levels must be 0-8\"}", "application/json");
        return;
      }
      extra_zoom_levels_ = v;
      std::string cur = GetCurrentMapName();
      RegenerateTiles(cur.empty() ? default_tiles_dir_ : GetTilesDir(cur));
      res.set_content("{\"extra_zoom_levels\":" + std::to_string(extra_zoom_levels_) + "}",
          "application/json");
    } catch (const std::exception& e) {
      res.status = 400;
      res.set_content("{\"error\":\"invalid json\"}", "application/json");
    }
  });

  // GET /getAllMapList
  // 返回 ~/.map 目录下所有地图名列表，子目录存在 map.yaml 即视为有效地图。
  svr.Get("/getAllMapList", [this](const httplib::Request&, httplib::Response& res) {
    res.set_header("Content-Type", "application/json");
    std::vector<std::string> names = GetAllMapNames(map_root_);
    nlohmann::json j = nlohmann::json::array();
    for (const auto& n : names) {
      j.push_back(n);
    }
    res.set_content(j.dump(), "application/json");
  });

  // GET /deleteMap?map_name=xxx
  // 删除 ~/.map/{map_name} 目录及其内容。
  svr.Get("/deleteMap", [this](const httplib::Request& req, httplib::Response& res) {
    res.set_header("Content-Type", "application/json");
    std::string map_name;
    if (!req.has_param("map_name") || (map_name = req.get_param_value("map_name")).empty()) {
      res.status = 400;
      res.set_content("{\"error\":\"missing map_name param\"}", "application/json");
      return;
    }
    if (map_name.find("..") != std::string::npos || map_name.find('/') != std::string::npos) {
      res.status = 400;
      res.set_content("{\"error\":\"invalid map_name\"}", "application/json");
      return;
    }
    std::string dir_path = GetMapDir(map_name);
    if (!fs::exists(dir_path)) {
      res.status = 404;
      res.set_content("{\"error\":\"map not found\"}", "application/json");
      return;
    }
    try {
      fs::remove_all(dir_path);
      res.set_content("{\"result\":\"ok\"}", "application/json");
    } catch (const std::exception& e) {
      res.status = 500;
      res.set_content(std::string("{\"error\":\"") + e.what() + "\"}", "application/json");
    }
  });

  // GET /getTopologyMap
  // 返回拓扑地图 JSON。无参数时返回当前内存中的拓扑；query map_name=xxx 时从 ~/.map/{map_name}/map.topology 读取。
  svr.Get("/getTopologyMap", [this](const httplib::Request& req, httplib::Response& res) {
    res.set_header("Content-Type", "application/json");
    std::string map_name;
    if (req.has_param("map_name")) {
      map_name = req.get_param_value("map_name");
    }
    TopologyMap topo;
    if (map_name.empty()) {
      topo = topo_map_;
    } else {
      std::string json_path = GetMapDir(map_name) + "/map.topology";
      if (!fs::exists(json_path)) {
        res.status = 404;
        res.set_content("{\"error\":\"topology not found\"}", "application/json");
        return;
      }
      if (LoadTopologyMapFromJson(json_path, topo) != LOAD_MAP_SUCCESS) {
        res.status = 500;
        res.set_content("{\"error\":\"failed to load topology\"}", "application/json");
        return;
      }
    }
    nlohmann::json j = topo;
    res.set_content(j.dump(), "application/json");
  });

  // GET /currentMap
  // 返回当前使用的地图名，来自 ~/.map/current_map。
  svr.Get("/currentMap", [this](const httplib::Request&, httplib::Response& res) {
    res.set_header("Content-Type", "application/json");
    nlohmann::json j;
    j["current_map"] = GetCurrentMapName();
    res.set_content(j.dump(), "application/json");
  });

  // GET /setCurrentMap?name=xxx
  // 切换当前地图，加载 ~/.map/{name}/map.yaml，并写入 current_map。
  svr.Get("/setCurrentMap", [this](const httplib::Request& req, httplib::Response& res) {
    res.set_header("Content-Type", "application/json");
    std::string name;
    if (!req.has_param("name") || (name = req.get_param_value("name")).empty()) {
      res.status = 400;
      res.set_content("{\"error\":\"missing name param\"}", "application/json");
      LOG_ERROR("setCurrentMap missing name param");
      return;
    }
    std::string base = GetMapDir(name);
    std::string yaml_path = base + "/" + name + ".yaml";
    std::ifstream check(yaml_path);
    if (!check) {
      res.status = 404;
      res.set_content("{\"error\":\"map not found\"}", "application/json");
      LOG_ERROR("setCurrentMap map not found: " << yaml_path);
      return;
    }
    LOAD_MAP_STATUS status = LoadMapFromYaml(yaml_path);
    if (status != LOAD_MAP_SUCCESS) {
      res.status = 500;
      res.set_content("{\"error\":\"failed to load map\"}", "application/json");
      LOG_ERROR("setCurrentMap failed to load map: " << yaml_path);
      return;
    }
    if (!SetCurrentMapName(name)) {
      res.status = 500;
      res.set_content("{\"error\":\"failed to set current map\"}", "application/json");
      return;
    }
    res.set_content("{\"result\":\"ok\"}", "application/json");
  });

  int max_retries = 100;
  while (http_running_ && max_retries > 0) {
    if (svr.bind_to_port("0.0.0.0", tiles_http_port_)) {
      http_svr_ = &svr;
      svr.listen_after_bind();
      http_svr_ = nullptr;
      break;
    }
    --max_retries;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

LOAD_MAP_STATUS MapManager::LoadMapFromYaml(const std::string& yaml_file) {
  LOAD_MAP_STATUS status = loadMapFromYaml(yaml_file, map_data_);
  if (status != LOAD_MAP_SUCCESS) {
    return status;
  }
  std::string json_file = yaml_file;
  size_t pos = json_file.rfind(".yaml");
  if (pos != std::string::npos) {
    json_file.replace(pos, 5, ".topology");
  }
  topo_map_file_name_ = json_file;
  LoadTopologyMapFromJson(json_file, topo_map_);
  map_available_ = true;
  size_t slash_pos = yaml_file.find_last_of("/\\");
  std::string map_dir = (slash_pos != std::string::npos && slash_pos > 0)
      ? yaml_file.substr(0, slash_pos) : ".";
  if (on_map_update_cb_) {
    on_map_update_cb_();
  }
  return LOAD_MAP_SUCCESS;
}

void MapManager::UpdateMap(const OccupancyGridData& data) {
  map_data_ = data;
  map_available_ = true;
  if (GetCurrentMapName().empty()) {
    SetCurrentMapName("map");
  }
  std::string current = GetCurrentMapName();
    if (!current.empty()) {
    fs::create_directories(fs::path(GetTilesDir(current)));
    SaveParameters save_params;
    save_params.map_file_name = GetMapDir(current) + "/map";
    save_params.image_format = "pgm";
    save_params.free_thresh = 0.25;
    save_params.occupied_thresh = 0.65;
    save_params.mode = MapMode::Trinary;
    saveMapToFile(map_data_, save_params);
  }
  std::string out_dir = current.empty() ? default_tiles_dir_ : GetTilesDir(current);
  RegenerateTiles(out_dir);
  LOG_INFO("Map updated, regenerated tiles");
  if (on_map_update_cb_) {
    on_map_update_cb_();
  }
}

void MapManager::UpdateTopoMap(const TopologyMap& data) {
  topo_map_ = data;
  if (GetCurrentMapName().empty()) {
    SetCurrentMapName("map");
  }
  std::string current = GetCurrentMapName();
  if (!current.empty()) {
    fs::create_directories(fs::path(GetTilesDir(current)));
    saveTopologyMapToJson(topo_map_, GetMapDir(current) + "/" + current + ".topology");
  }
  if (on_map_update_cb_) {
    on_map_update_cb_();
  }
}

}  // namespace nav2_map_server
