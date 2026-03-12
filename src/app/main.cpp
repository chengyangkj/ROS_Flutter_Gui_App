#include "node/map_server_interface.hpp"
#include "common/logger/logger.h"

#include <boost/program_options.hpp>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>

#if ROS_VERSION == 1
#include "node/ros1/map_server_node.hpp"
#else
#include "node/ros2/map_server_node.hpp"
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
      ("tiles_http_port", po::value<int>()->default_value(8080),
       "HTTP port for serving tiles to flutter_map")
      ("tiles_output_dir", po::value<std::string>()->default_value("./tiles"),
       "local directory for tile cache, served via HTTP")
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
  config.tiles_output_dir = vm["tiles_output_dir"].as<std::string>();
  config.save_map_timeout = vm["save_map_timeout"].as<double>();
  config.free_thresh_default = vm["free_thresh_default"].as<double>();
  config.occupied_thresh_default = vm["occupied_thresh_default"].as<double>();
  config.map_subscribe_transient_local = vm["map_subscribe_transient_local"].as<bool>();
  return config;
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
  ros::init(argc, argv, "map_server");
  auto node = std::make_shared<nav2_map_server::MapServerNode>();
  if (!node->Init(config)) {
    return -1;
  }
  node->Run();
  node->Shutdown();
#else
  rclcpp::init(argc, argv);
  auto node = std::make_shared<nav2_map_server::MapServerNode>(config);
  node->Run();
  node->Shutdown();
  rclcpp::shutdown();
#endif
  return 0;
}
