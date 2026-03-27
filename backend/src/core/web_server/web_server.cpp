#include "core/web_server/web_server.hpp"
#include "core/map/map_manager.hpp"
#include "core/map/map_io.hpp"
#include "core/map/tiles_map_generator.h"
#include "core/map/json.hpp"
#include "core/robot_stream/robot_message_hub.hpp"
#include "core/robot_stream/robot_ws_controller.hpp"
#include "core/robot_stream/ssh_tunnel_ws_controller.hpp"
#include "common/config/config.hpp"
#include "node/interface.hpp"
#include "node/node_manager.hpp"
#include "common/logger/logger.h"
#include "robot_message.pb.h"

#include <drogon/drogon.h>
#include <chrono>
#include <boost/filesystem.hpp>
#include <fstream>
#include <string>
#include <vector>

#if defined(__linux__)
#include <unistd.h>
#endif

namespace fs = boost::filesystem;

using namespace std::placeholders;

namespace ros_gui_backend {

namespace {

std::function<void()> g_sigint_hook;

drogon::HttpStatusCode HttpStatusForMapError(const std::string& err) {
  if (err == "map not available") {
    return drogon::k503ServiceUnavailable;
  }
  if (err == "map not found") {
    return drogon::k404NotFound;
  }
  if (err == "failed to save map") {
    return drogon::k500InternalServerError;
  }
  return drogon::k400BadRequest;
}

std::string JsonErrorBody(const std::string& msg) {
  nlohmann::json j;
  j["error"] = msg;
  return j.dump();
}

std::string ExecutableDirectory() {
#if defined(__linux__)
  char buf[4096];
  const ssize_t n = readlink("/proc/self/exe", buf, sizeof(buf) - 1);
  if (n > 0) {
    buf[n] = '\0';
    return fs::path(buf).parent_path().string();
  }
#endif
  return fs::current_path().string();
}

std::string ResolvedDocumentRoot(const WebServerConfig& cfg) {
  const std::string base = ExecutableDirectory();
  if (!cfg.document_root.empty()) {
    fs::path p(cfg.document_root);
    if (p.is_absolute()) {
      return p.string();
    }
    return (fs::path(base) / p).string();
  }
  return (fs::path(base) / "dist").string();
}

}  // namespace

WebServer::WebServer() = default;

WebServer::~WebServer() {
  Shutdown();
}

void WebServer::SetSigintHook(std::function<void()> hook) {
  g_sigint_hook = std::move(hook);
}

void WebServer::ClearSigintHook() {
  g_sigint_hook = nullptr;
}

bool WebServer::Start(const WebServerConfig& config) {
  if (running_) {
    return true;
  }
  running_ = true;
  thread_ = std::make_unique<std::thread>(&WebServer::RunImpl, this, config);
  return true;
}

void WebServer::Shutdown() {
  if (!running_) {
    return;
  }
  running_ = false;
  drogon::app().quit();
  if (thread_ && thread_->joinable()) {
    thread_->join();
  }
  thread_.reset();
}

void WebServer::RunImpl(WebServerConfig config) {
  using drogon::Get;
  using drogon::HttpRequestPtr;
  using drogon::HttpResponse;
  using drogon::HttpResponsePtr;
  using drogon::Post;



  auto add_cors = [](const HttpResponsePtr& resp) {
    resp->addHeader("Access-Control-Allow-Origin", "*");
    resp->addHeader("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
    resp->addHeader(
        "Access-Control-Allow-Headers", "Content-Type, Accept, Authorization");
  };

  auto json_cb = [&add_cors](std::function<void(const HttpResponsePtr&)>&& callback, const std::string& body,
                     drogon::HttpStatusCode code) {
    auto r = HttpResponse::newHttpResponse();
    r->setStatusCode(code);
    r->setContentTypeCode(drogon::CT_APPLICATION_JSON);
    r->setBody(body);
    add_cors(r);
    callback(r);
  };

  auto plain_cb = [&add_cors](std::function<void(const HttpResponsePtr&)>&& callback, const std::string& body,
                      drogon::HttpStatusCode code, const char* ctype = "text/plain; charset=utf-8") {
    auto r = HttpResponse::newHttpResponse();
    r->setStatusCode(code);
    r->setContentTypeString(ctype);
    r->setBody(body);
    add_cors(r);
    callback(r);
  };

  auto serve_tile = [&plain_cb, &add_cors](const std::string& tiles_dir, const std::string& z, const std::string& x,
                       std::string y, std::function<void(const HttpResponsePtr&)>&& callback) {
    if (y.size() >= 4 && y.compare(y.size() - 4, 4, ".png") == 0) {
      y.resize(y.size() - 4);
    }
    std::string path = tiles_dir + "/" + z + "/" + x + "/" + y + ".png";
    std::ifstream ifs(path, std::ios::binary);
    if (!ifs) {
      LOGGER_WARN("Tile not found z={} x={} y={} path={}", z, x, y, path);
      plain_cb(std::move(callback), "Not found", drogon::k404NotFound);
      return;
    }
    ifs.seekg(0, std::ios::end);
    auto pos = ifs.tellg();
    ifs.seekg(0, std::ios::beg);
    if (pos < 0 || static_cast<size_t>(pos) > 512 * 1024) {
      plain_cb(std::move(callback), "Invalid tile", drogon::k500InternalServerError);
      return;
    }
    size_t size = static_cast<size_t>(pos);
    std::string content(size, '\0');
    if (!ifs.read(&content[0], size)) {
      plain_cb(std::move(callback), "Read error", drogon::k500InternalServerError);
      return;
    }
    auto resp = HttpResponse::newHttpResponse();
    resp->setStatusCode(drogon::k200OK);
    resp->setContentTypeString("image/png");
    resp->setBody(std::move(content));
    add_cors(resp);
    callback(resp);
  };

  drogon::app().registerSyncAdvice([add_cors](const HttpRequestPtr& req) -> HttpResponsePtr {
    if (req->path() == "/ws/robot" || req->path() == "/ws/ssh") {
      LOGGER_INFO("ws {} HTTP {} upgrade={} peer={}", req->path(), req->getMethodString(),
          req->getHeader("upgrade"), req->getPeerAddr().toIpPort());
    }
    if (req->getMethod() == drogon::HttpMethod::Options) {
      auto r = HttpResponse::newHttpResponse();
      r->setStatusCode(drogon::k204NoContent);
      add_cors(r);
      r->addHeader("Access-Control-Max-Age", "86400");
      return r;
    }
    return nullptr;
  });

  drogon::app().registerHandler(
      "/subImage",
      [&json_cb](const HttpRequestPtr& req, std::function<void(const HttpResponsePtr&)>&& callback) {
        const auto& body = req->getBody();
        nlohmann::json j;
        try {
          j = body.empty() ? nlohmann::json::object() : nlohmann::json::parse(body);
        } catch (const std::exception&) {
          json_cb(std::move(callback), JsonErrorBody("invalid json body"), drogon::k400BadRequest);
          return;
        }
        bool want_subscribe = false;
        std::string topic;
        try {
          want_subscribe = j.at("subscribe").get<bool>();
          topic = j.value("topic", std::string{});
        } catch (const nlohmann::json::exception&) {
          json_cb(std::move(callback), JsonErrorBody("expect {\"topic\": string, \"subscribe\": bool}"),
              drogon::k400BadRequest);
          return;
        }
        LOGGER_INFO("POST /subImage topic={} subscribe={}", topic, want_subscribe);
        if (want_subscribe) {
          if (topic.empty()) {
            json_cb(std::move(callback), JsonErrorBody("topic required when subscribing"),
                drogon::k400BadRequest);
            return;
          }
          std::string err;
          if (!NodeManager::Instance()->SetRobotStreamImageSubscription(topic, true, &err)) {
            const drogon::HttpStatusCode code =
                err.find("not found") != std::string::npos ? drogon::k404NotFound
                                                           : drogon::k400BadRequest;
            json_cb(std::move(callback), JsonErrorBody(err), code);
            return;
          }
        } else {
          NodeManager::Instance()->SetRobotStreamImageSubscription(topic, false, nullptr);
        }
        json_cb(std::move(callback), "{}", drogon::k200OK);
      },
      {Post});

  drogon::app().registerHandler(
      "/api/settings",
      [&json_cb](const HttpRequestPtr& req, std::function<void(const HttpResponsePtr&)>&& callback) {
        if (req->getMethod() == drogon::Get) {
          LOGGER_INFO("GET /api/settings");
          GuiAppSettings s{};
          LoadGuiAppSettingsFile(&s);
          nlohmann::json j;
          GuiAppSettingsToJson(s, &j);
          json_cb(std::move(callback), j.dump(), drogon::k200OK);
          return;
        }
        LOGGER_INFO("POST /api/settings");
        nlohmann::json body;
        try {
          body = req->getBody().empty() ? nlohmann::json::object() : nlohmann::json::parse(req->getBody());
        } catch (const std::exception&) {
          json_cb(std::move(callback), JsonErrorBody("invalid json"), drogon::k400BadRequest);
          return;
        }
        GuiAppSettings s{};
        LoadGuiAppSettingsFile(&s);
        GuiAppSettingsMergeJson(body, &s);
        if (!SaveGuiAppSettingsFile(s)) {
          json_cb(std::move(callback), JsonErrorBody("failed to save settings"), drogon::k500InternalServerError);
          return;
        }
        Config::Instance()->MutableGuiApp() = s;
        if (auto node = NodeManager::Instance()->GetNode()) {
          node->ReloadGuiStreams(s);
        }
        nlohmann::json out;
        GuiAppSettingsToJson(s, &out);
        json_cb(std::move(callback), out.dump(), drogon::k200OK);
      },
      {Get, Post});

  drogon::app().registerHandler(
      "/api/tf",
      [&json_cb](const HttpRequestPtr& req, std::function<void(const HttpResponsePtr&)>&& callback) {
        std::string target = req->getParameter("target_frame");
        std::string source = req->getParameter("source_frame");
        LOGGER_INFO("GET /api/tf target={} source={}", target, source);
        if (target.empty() || source.empty()) {
          json_cb(std::move(callback), JsonErrorBody("query target_frame and source_frame required"),
              drogon::k400BadRequest);
          return;
        }
        auto node = NodeManager::Instance()->GetNode();
        if (!node) {
          json_cb(std::move(callback), JsonErrorBody("ros node not ready"), drogon::k503ServiceUnavailable);
          return;
        }
        std::string jout;
        std::string err;
        if (!node->LookupTransform(target, source, &jout, &err)) {
          json_cb(std::move(callback), JsonErrorBody(err), drogon::k404NotFound);
          return;
        }
        json_cb(std::move(callback), jout, drogon::k200OK);
      },
      {Get});

  auto robot_post_json = [&json_cb](const char* path,
                             std::function<bool(const std::shared_ptr<IRosGuiNode>&, const nlohmann::json&)>&& fn) {
    return [path, fn, &json_cb](const HttpRequestPtr& req, std::function<void(const HttpResponsePtr&)>&& callback) {
      LOGGER_INFO("POST {}", path);
      auto node = NodeManager::Instance()->GetNode();
      if (!node) {
        json_cb(std::move(callback), JsonErrorBody("ros node not ready"), drogon::k503ServiceUnavailable);
        return;
      }
      nlohmann::json j;
      try {
        j = req->getBody().empty() ? nlohmann::json::object() : nlohmann::json::parse(req->getBody());
      } catch (const std::exception&) {
        json_cb(std::move(callback), JsonErrorBody("invalid json"), drogon::k400BadRequest);
        return;
      }
      if (!fn(node, j)) {
        json_cb(std::move(callback), JsonErrorBody("command failed"), drogon::k400BadRequest);
        return;
      }
      json_cb(std::move(callback), "{}", drogon::k200OK);
    };
  };

  drogon::app().registerHandler(
      "/robot/cmd_vel",
      robot_post_json("/robot/cmd_vel",
          [](const std::shared_ptr<IRosGuiNode>& node, const nlohmann::json& j) {
            const double vx = j.value("vx", 0.0);
            const double vy = j.value("vy", 0.0);
            const double vw = j.value("vw", 0.0);
            return node->PublishCmdVel(vx, vy, vw);
          }),
      {Post});

  drogon::app().registerHandler(
      "/robot/nav_goal",
      robot_post_json("/robot/nav_goal",
          [](const std::shared_ptr<IRosGuiNode>& node, const nlohmann::json& j) {
            const double x = j.at("x").get<double>();
            const double y = j.at("y").get<double>();
            const double yaw = j.value("yaw", j.value("theta", 0.0));
            const double roll = j.value("roll", 0.0);
            const double pitch = j.value("pitch", 0.0);
            return node->PublishNavGoal(x, y, roll, pitch, yaw);
          }),
      {Post});

  drogon::app().registerHandler(
      "/robot/initial_pose",
      robot_post_json("/robot/initial_pose",
          [](const std::shared_ptr<IRosGuiNode>& node, const nlohmann::json& j) {
            const double x = j.at("x").get<double>();
            const double y = j.at("y").get<double>();
            const double yaw = j.value("yaw", j.value("theta", 0.0));
            const double roll = j.value("roll", 0.0);
            const double pitch = j.value("pitch", 0.0);
            return node->PublishInitialPose(x, y, roll, pitch, yaw);
          }),
      {Post});

  drogon::app().registerHandler(
      "/robot/cancel_nav",
      [&json_cb](const HttpRequestPtr&, std::function<void(const HttpResponsePtr&)>&& callback) {
        LOGGER_INFO("POST /robot/cancel_nav");
        auto node = NodeManager::Instance()->GetNode();
        if (!node) {
          json_cb(std::move(callback), JsonErrorBody("ros node not ready"), drogon::k503ServiceUnavailable);
          return;
        }
        if (!node->PublishNavCancel()) {
          json_cb(std::move(callback), JsonErrorBody("cancel failed"), drogon::k400BadRequest);
          return;
        }
        json_cb(std::move(callback), "{}", drogon::k200OK);
      },
      {Post});

  drogon::app().registerHandler(
      "/getAllMapList",
      [&json_cb](const HttpRequestPtr&, std::function<void(const HttpResponsePtr&)>&& callback) {
        LOGGER_INFO("GET /getAllMapList");
        MapManager* m = MapManager::Instance();
        std::vector<std::string> names = m->ListMapNames();
        nlohmann::json j = nlohmann::json::array();
        for (const auto& n : names) {
          j.push_back(n);
        }
        json_cb(std::move(callback), j.dump(), drogon::k200OK);
      },
      {Get});

  drogon::app().registerHandler(
      "/deleteMap",
      [&json_cb](const HttpRequestPtr& req, std::function<void(const HttpResponsePtr&)>&& callback) {
        LOGGER_INFO("GET /deleteMap map_name={}", req->getParameter("map_name"));
        MapManager* m = MapManager::Instance();
        std::string map_name = req->getParameter("map_name");
        if (map_name.empty()) {
          json_cb(std::move(callback), "{\"error\":\"missing map_name param\"}", drogon::k400BadRequest);
          return;
        }
        if (map_name.find("..") != std::string::npos || map_name.find('/') != std::string::npos) {
          json_cb(std::move(callback), "{\"error\":\"invalid map_name\"}", drogon::k400BadRequest);
          return;
        }
        std::string dir_path = m->GetMapDir(map_name);
        if (!fs::exists(dir_path)) {
          json_cb(std::move(callback), "{\"error\":\"map not found\"}", drogon::k404NotFound);
          return;
        }
        try {
          fs::remove_all(dir_path);
          json_cb(std::move(callback), "{\"result\":\"ok\"}", drogon::k200OK);
        } catch (const std::exception& e) {
          json_cb(std::move(callback), std::string("{\"error\":\"") + e.what() + "\"}",
              drogon::k500InternalServerError);
        }
      },
      {Get});

  drogon::app().registerHandler(
      "/getTopologyMap",
      [&json_cb](const HttpRequestPtr& req, std::function<void(const HttpResponsePtr&)>&& callback) {
        LOGGER_INFO("GET /getTopologyMap map_name={}", req->getParameter("map_name"));
        MapManager* m = MapManager::Instance();
        std::string map_name = req->getParameter("map_name");
        TopologyMap topo;
        if (map_name.empty()) {
          topo = m->GetTopoMap();
        } else {
          std::string json_path = m->GetMapDir(map_name) + "/map.topology";
          if (!fs::exists(json_path)) {
            json_cb(std::move(callback), "{\"error\":\"topology not found\"}", drogon::k404NotFound);
            return;
          }
          if (LoadTopologyMapFromJson(json_path, topo) != LOAD_MAP_SUCCESS) {
            json_cb(std::move(callback), "{\"error\":\"failed to load topology\"}",
                drogon::k500InternalServerError);
            return;
          }
        }
        nlohmann::json j = topo;
        json_cb(std::move(callback), j.dump(), drogon::k200OK);
      },
      {Get});

  drogon::app().registerHandler(
      "/currentMap",
      [&json_cb](const HttpRequestPtr&, std::function<void(const HttpResponsePtr&)>&& callback) {
        LOGGER_INFO("GET /currentMap");
        MapManager* m = MapManager::Instance();
        nlohmann::json j;
        j["current_map"] = m->GetCurrentMapName();
        json_cb(std::move(callback), j.dump(), drogon::k200OK);
      },
      {Get});

  drogon::app().registerHandler(
      "/setCurrentMap",
      [&json_cb](const HttpRequestPtr& req, std::function<void(const HttpResponsePtr&)>&& callback) {
        LOGGER_INFO("GET /setCurrentMap name={}", req->getParameter("name"));
        MapManager* m = MapManager::Instance();
        std::string name = req->getParameter("name");
        if (name.empty()) {
          json_cb(std::move(callback), "{\"error\":\"missing name param\"}", drogon::k400BadRequest);
          LOGGER_ERROR("setCurrentMap missing name param");
          return;
        }
        std::string base = m->GetMapDir(name);
        std::string yaml_path = base + "/" + name + ".yaml";
        std::ifstream check(yaml_path);
        if (!check) {
          json_cb(std::move(callback), "{\"error\":\"map not found\"}", drogon::k404NotFound);
          LOGGER_ERROR("setCurrentMap map not found: {}", yaml_path);
          return;
        }
        LOAD_MAP_STATUS status = m->LoadMapFromYaml(yaml_path);
        if (status != LOAD_MAP_SUCCESS) {
          json_cb(std::move(callback), "{\"error\":\"failed to load map\"}", drogon::k500InternalServerError);
          LOGGER_ERROR("setCurrentMap failed to load map: {}", yaml_path);
          return;
        }
        if (!m->SetCurrentMapName(name)) {
          json_cb(std::move(callback), "{\"error\":\"failed to set current map\"}",
              drogon::k500InternalServerError);
          return;
        }
        json_cb(std::move(callback), "{\"result\":\"ok\"}", drogon::k200OK);
      },
      {Get});

  drogon::app().registerHandler(
      "/updateMapEdit",
      [&json_cb](const HttpRequestPtr& req, std::function<void(const HttpResponsePtr&)>&& callback) {
        MapManager* m = MapManager::Instance();
        auto [ok, err] = m->ApplyMapEditFromQuery(req->getParameter("session_id"),
            req->getParameter("map_name"), req->getParameter("topology_json"),
            req->getParameter("obstacle_edits_json"));
        if (!ok) {
          json_cb(std::move(callback), JsonErrorBody(err), HttpStatusForMapError(err));
          return;
        }
        json_cb(std::move(callback), "{\"result\":\"ok\"}", drogon::k200OK);
      },
      {Get});

  drogon::app().registerHandler(
      "/tiles/",
      [&plain_cb](const HttpRequestPtr&, std::function<void(const HttpResponsePtr&)>&& callback) {
        LOGGER_INFO("GET /tiles/");
        plain_cb(std::move(callback),
            "Tiles server for flutter_map.\n"
            "Tiles: /tiles/{z}/{x}/{y}.png (current) or /tiles/{map_name}/{z}/{x}/{y}.png\n"
            "Meta: /tiles/meta (current) or /tiles/{map_name}/meta\n"
            "Config: POST /tiles/config or POST /tiles/{map_name}/config\n"
            "Map list: GET /getAllMapList\n"
            "Delete map: GET /deleteMap?map_name=xxx\n"
            "Topology: GET /getTopologyMap or GET /getTopologyMap?map_name=xxx",
            drogon::k200OK, "text/plain; charset=utf-8");
      },
      {Get});

  drogon::app().registerHandler(
      "/tiles/meta",
      [&json_cb](const HttpRequestPtr&, std::function<void(const HttpResponsePtr&)>&& callback) {
        LOGGER_INFO("GET /tiles/meta");
        MapManager* m = MapManager::Instance();
        std::string meta;
        if (!m->TryBuildCurrentTilesMetaJson(&meta)) {
          json_cb(std::move(callback), "{\"error\":\"map not available\"}", drogon::k503ServiceUnavailable);
          return;
        }
        json_cb(std::move(callback), meta, drogon::k200OK);
      },
      {Get});

  drogon::app().registerHandler(
      "/tiles/{map_name}/meta",
      [&json_cb](const HttpRequestPtr&, std::function<void(const HttpResponsePtr&)>&& callback,
          const std::string& map_name) {
        LOGGER_INFO("GET /tiles/{}/meta", map_name);
        MapManager* m = MapManager::Instance();
        std::string yaml_path = m->GetMapDir(map_name) + "/map.yaml";
        if (!fs::exists(yaml_path)) {
          json_cb(std::move(callback), "{\"error\":\"map not found\"}", drogon::k404NotFound);
          return;
        }
        double resolution = 0, origin_x = 0, origin_y = 0, origin_yaw = 0;
        uint32_t width = 0, height = 0;
        if (!m->ReadYamlMapMeta(yaml_path, resolution, origin_x, origin_y, origin_yaw, width, height)) {
          json_cb(std::move(callback), "{\"error\":\"failed to read map meta\"}",
              drogon::k500InternalServerError);
          return;
        }
        TilesMapGenerator gen;
        int max_zoom = gen.GetMaxZoom(width, height, m->GetExtraZoomLevels());
        nlohmann::json j;
        j["resolution"] = resolution;
        j["origin_x"] = origin_x;
        j["origin_y"] = origin_y;
        j["width"] = width;
        j["height"] = height;
        j["max_zoom"] = max_zoom;
        j["extra_zoom_levels"] = m->GetExtraZoomLevels();
        json_cb(std::move(callback), j.dump(), drogon::k200OK);
      },
      {Get});

  drogon::app().registerHandler(
      "/tiles/config",
      [&json_cb](const HttpRequestPtr& req, std::function<void(const HttpResponsePtr&)>&& callback) {
        LOGGER_INFO("POST /tiles/config");
        MapManager* m = MapManager::Instance();
        auto [ok, err] = m->ApplyTilesExtraZoomFromJson(req->getBody());
        if (!ok) {
          json_cb(std::move(callback), JsonErrorBody(err), HttpStatusForMapError(err));
          return;
        }
        nlohmann::json j;
        j["extra_zoom_levels"] = m->GetExtraZoomLevels();
        json_cb(std::move(callback), j.dump(), drogon::k200OK);
      },
      {Post});

  drogon::app().registerHandler(
      "/tiles/{map_name}/config",
      [&json_cb](const HttpRequestPtr& req, std::function<void(const HttpResponsePtr&)>&& callback,
          const std::string& map_name) {
        LOGGER_INFO("POST /tiles/{}/config", map_name);
        MapManager* m = MapManager::Instance();
        auto [ok, err] = m->ApplyTilesExtraZoomForMapYaml(map_name, req->getBody());
        if (!ok) {
          json_cb(std::move(callback), JsonErrorBody(err), HttpStatusForMapError(err));
          return;
        }
        nlohmann::json j;
        j["extra_zoom_levels"] = m->GetExtraZoomLevels();
        json_cb(std::move(callback), j.dump(), drogon::k200OK);
      },
      {Post});

  drogon::app().registerHandler(
      "/tiles/{map_name}/{z}/{x}/{y}",
      [serve_tile](const HttpRequestPtr&, std::function<void(const HttpResponsePtr&)>&& callback,
          const std::string& map_name, const std::string& z, const std::string& x, const std::string& y) {
        LOGGER_INFO("GET /tiles/{}/{}/{}/{}", map_name, z, x, y);
        MapManager* m = MapManager::Instance();
        serve_tile(m->GetTilesDir(map_name), z, x, y, std::move(callback));
      },
      {Get});

  drogon::app().registerHandler(
      "/tiles/{z}/{x}/{y}",
      [serve_tile](const HttpRequestPtr&, std::function<void(const HttpResponsePtr&)>&& callback,
          const std::string& z, const std::string& x, const std::string& y) {
        LOGGER_INFO("GET /tiles/{}/{}/{}", z, x, y);
        MapManager* m = MapManager::Instance();
        std::string current = m->GetCurrentMapName();
        std::string tiles_dir =
            current.empty() ? m->GetDefaultTilesDir() : m->GetTilesDir(current);
        serve_tile(tiles_dir, z, x, y, std::move(callback));
      },
      {Get});

  const std::string doc_root = ResolvedDocumentRoot(config);
  if (fs::is_directory(doc_root)) {
    drogon::app().setDocumentRoot(doc_root);
    drogon::app().setFileTypes({"html", "htm", "js", "mjs", "css", "json", "png", "jpg", "jpeg", "gif",
        "svg", "ico", "wasm", "txt", "map", "woff", "woff2", "ttf", "eot", "otf", "webmanifest"});
    drogon::app().setDefaultHandler(
        [&json_cb, &add_cors](const HttpRequestPtr& req, std::function<void(const HttpResponsePtr&)>&& callback) {
          const auto method = req->getMethod();
          if (method != drogon::Get && method != drogon::Head) {
            json_cb(std::move(callback), JsonErrorBody("not found"), drogon::k404NotFound);
            return;
          }
          const std::string& root = drogon::app().getDocumentRoot();
          const std::string index_path = (fs::path(root) / "index.html").string();
          if (!fs::is_regular_file(index_path)) {
            json_cb(std::move(callback), JsonErrorBody("index.html missing"), drogon::k503ServiceUnavailable);
            return;
          }
          const HttpResponsePtr resp = HttpResponse::newFileResponse(index_path);
          add_cors(resp);
          callback(resp);
        });
    LOGGER_INFO("HTTP static files from {}", doc_root);
  } else {
    LOGGER_WARN("HTTP document_root is not a directory, static UI disabled: {}", doc_root);
  }

  LOGGER_INFO(
      "Tiles HTTP server at http://0.0.0.0:{}, URL: /tiles/{{z}}/{{x}}/{{y}}.png", config.port);
  drogon::app().addListener("0.0.0.0", static_cast<uint16_t>(config.port));
  drogon::app().setThreadNum(1);
  drogon::app().setIntSignalHandler([]() {
    if (g_sigint_hook) {
      g_sigint_hook();
    }
    drogon::app().quit();
  });

  drogon::app().getLoop()->runEvery(
      5.0,
      []() {
        static uint32_t seq = 0;
        ros_gui_backend::pb::RobotMessage msg;
        ros_gui_backend::pb::Heartbeat* hb = msg.mutable_heartbeat();
        const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch());
        hb->set_server_time_ms(ms.count());
        hb->set_seq(++seq);
        RobotMessageHub::Instance().BroadcastRobotMsg(msg);
      });

  drogon::app().run();
}

}  // namespace ros_gui_backend
