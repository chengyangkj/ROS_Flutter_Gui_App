#include "core/map_server_core.hpp"
#include "core/tiles_map_generator.h"

#include "common/logger/logger.h"

#include <httplib.h>
#include "core/json.hpp"

#include <fstream>

namespace nav2_map_server {

MapServerCore::MapServerCore() : map_available_(false) {}

MapServerCore::~MapServerCore() {
  Shutdown();
}

bool MapServerCore::Initialize(int tiles_http_port,
    const std::string& tiles_output_dir) {
  tiles_http_port_ = tiles_http_port;
  tiles_output_dir_ = tiles_output_dir;
  return true;
}

void MapServerCore::Start() {
  if (http_running_) return;
  http_running_ = true;
  http_thread_ = std::make_unique<std::thread>(&MapServerCore::RunHttpServer, this);
  LOG_INFO("Tiles HTTP server started at http://0.0.0.0:" << tiles_http_port_
      << ", serving from " << tiles_output_dir_ << ", URL: /tiles/{z}/{x}/{y}.png");
}

void MapServerCore::Shutdown() {
  http_running_ = false;
  if (auto* s = http_svr_.exchange(nullptr)) {
    s->stop();
  }
  if (http_thread_ && http_thread_->joinable()) {
    http_thread_->join();
  }
  http_thread_.reset();
}

void MapServerCore::RegenerateTiles() {
  if (!map_available_) return;
  TilesMapGenerator gen;
  if (gen.GenerateAllTilesToDir(map_data_, tiles_output_dir_, extra_zoom_levels_)) {
    LOG_INFO("Tiles regenerated to " << tiles_output_dir_);
  }
}

void MapServerCore::RunHttpServer() {
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

  svr.Get("/tiles/:z/:x/:y.png", [this](const httplib::Request& req, httplib::Response& res) {
    std::string z = req.path_params.at("z");
    std::string x = req.path_params.at("x");
    std::string y = req.path_params.at("y.png");
    if (y.size() >= 4 && y.compare(y.size() - 4, 4, ".png") == 0) {
      y.resize(y.size() - 4);
    }
    std::string path = tiles_output_dir_ + "/" + z + "/" + x + "/" + y + ".png";
    std::ifstream ifs(path, std::ios::binary);
    if (!ifs) {
      LOG_WARN("Tile not found req z=" << z << " x=" << x << " y=" << y << " path=" << path);
      res.status = 404;
      return;
    }
    ifs.seekg(0, std::ios::end);
    size_t size = ifs.tellg();
    ifs.seekg(0);
    std::string content(size, '\0');
    if (!ifs.read(&content[0], size)) {
      res.status = 500;
      return;
    }
    res.set_header("Content-Type", "image/png");
    res.set_content(content, "image/png");
  });

  svr.Get("/tiles/", [this](const httplib::Request&, httplib::Response& res) {
    res.set_content(
        "Tiles server for flutter_map.\n"
        "URL template: http://localhost:" + std::to_string(tiles_http_port_) + "/tiles/{z}/{x}/{y}.png",
        "text/plain");
  });

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
      RegenerateTiles();
      res.set_content("{\"extra_zoom_levels\":" + std::to_string(extra_zoom_levels_) + "}",
          "application/json");
    } catch (const std::exception& e) {
      res.status = 400;
      res.set_content("{\"error\":\"invalid json\"}", "application/json");
    }
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

LOAD_MAP_STATUS MapServerCore::LoadMapFromYaml(const std::string& yaml_file) {
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
  RegenerateTiles();
  return LOAD_MAP_SUCCESS;
}

void MapServerCore::UpdateMap(const OccupancyGridData& data) {
  map_data_ = data;
  map_available_ = true;
  RegenerateTiles();
  LOG_INFO("Map updated, regenerated tiles");
}

void MapServerCore::UpdateTopoMap(const TopologyMap& data) {
  topo_map_ = data;
}

}  // namespace nav2_map_server
