#include "node/map_server_interface.hpp"
#include "app/map_manager.hpp"
#include "common/logger/logger.h"

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>

#if ROS_VERSION == 1
#include "node/ros1/map_server_node.hpp"
#include <ros/ros.h>
#else
#include "node/ros2/map_server_node.hpp"
#include "rclcpp/rclcpp.hpp"
#endif

namespace po = boost::program_options;

nav2_map_server::MapServerConfig ParseOptions(int argc, char** argv) {
  po::options_description desc("Map server options");
  desc.add_options()
      ("yaml_filename,y", po::value<std::string>()->default_value("./map.yaml"),
       "yaml file path,which is default to load or save map path")
      ("pub_map_topic,t", po::value<std::string>()->default_value("/tiles/map"),
       "map publish topic name,which is read from yaml file")
      ("sub_map_topic", po::value<std::string>()->default_value("/map"),
       "map subscribe topic name,which is used to subscribe map from other nodes,will transform this topic top tile map")
      ("frame_id,f", po::value<std::string>()->default_value("map"),
       "frame id")
      ("tiles_http_port", po::value<int>()->default_value(7684),
       "HTTP port for serving tiles to flutter_map")
      ("save_map_timeout", po::value<double>()->default_value(2.0),
       "save map timeout in seconds")
      ("free_thresh_default", po::value<double>()->default_value(0.25),
       "free threshold default")
      ("occupied_thresh_default", po::value<double>()->default_value(0.65),
       "occupied threshold default")
      ("map_subscribe_transient_local", po::value<bool>()->default_value(true),
       "use transient local for map subscription")
      ("config,c", po::value<std::string>(),
       "config file");

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);

  std::string config_file;
  if (vm.count("config")) {
    config_file = vm["config"].as<std::string>();
    std::ifstream ifs(config_file);
    if (ifs) {
      po::store(po::parse_config_file(ifs, desc), vm);
    }
  }
  po::notify(vm);

  nav2_map_server::MapServerConfig config;
  config.yaml_filename = vm["yaml_filename"].as<std::string>();
  config.pub_map_topic = vm["pub_map_topic"].as<std::string>();
  config.sub_map_topic = vm["sub_map_topic"].as<std::string>();
  config.frame_id = vm["frame_id"].as<std::string>();
  config.tiles_http_port = vm["tiles_http_port"].as<int>();
  config.save_map_timeout = vm["save_map_timeout"].as<double>();
  config.free_thresh_default = vm["free_thresh_default"].as<double>();
  config.occupied_thresh_default = vm["occupied_thresh_default"].as<double>();
  config.map_subscribe_transient_local = vm["map_subscribe_transient_local"].as<bool>();
  return config;
}

static std::string ResolveInitYaml(const nav2_map_server::MapServerConfig& config,
    nav2_map_server::MapManager& map_manager) {
  if (!config.yaml_filename.empty() && boost::filesystem::exists(config.yaml_filename)) {
    return config.yaml_filename;
  }
  std::string current = map_manager.GetCurrentMapName();
  if (!current.empty()) {
    std::string yaml_path = map_manager.GetMapDir(current) + "/" + current + ".yaml";
    if (boost::filesystem::exists(yaml_path)) {
      return yaml_path;
    }
  }
  return std::string();
}

int main(int argc, char** argv) {
  nav2_map_server::MapServerConfig config;
  try {
    config = ParseOptions(argc, argv);
  } catch (const po::error& e) {
    LOG_ERROR("Options error: " << e.what());
    return -1;
  }

#if ROS_VERSION == 1
  ros::init(argc, argv, "map_manager");
  nav2_map_server::MapManager map_manager;
  map_manager.SetFrameId(config.frame_id);
  std::string init_yaml = ResolveInitYaml(config, map_manager);
  if (!map_manager.Initialize(config.tiles_http_port)) {
    LOG_ERROR("Failed to initialize map manager");
    return -1;
  }
  auto node = std::make_shared<nav2_map_server::MapServerNode>();
  if (!node->Init(config, &map_manager)) {
    return -1;
  }
  if (!init_yaml.empty()) {
    if (map_manager.LoadMapFromYaml(init_yaml) != nav2_map_server::LOAD_MAP_SUCCESS) {
      LOG_ERROR("Failed to load map: " << init_yaml);
      return -1;
    }
  }
  map_manager.Start();
  node->Run();
  map_manager.Shutdown();
  node->Shutdown();
#else
  rclcpp::init(argc, argv);
  nav2_map_server::MapManager map_manager;
  map_manager.SetFrameId(config.frame_id);
  std::string init_yaml = ResolveInitYaml(config, map_manager);
  if (!map_manager.Initialize(config.tiles_http_port)) {
    LOG_ERROR("Failed to initialize map manager");
    return -1;
  }
  auto node = std::make_shared<nav2_map_server::MapServerNode>(config);
  if (!node->Init(config, &map_manager)) {
    return -1;
  }
  if (!init_yaml.empty()) {
    if (map_manager.LoadMapFromYaml(init_yaml) != nav2_map_server::LOAD_MAP_SUCCESS) {
      LOG_ERROR("Failed to load map: " << init_yaml);
      return -1;
    }
  }
  map_manager.Start();
  node->Run();
  map_manager.Shutdown();
  node->Shutdown();
  rclcpp::shutdown();
#endif
  return 0;
}
