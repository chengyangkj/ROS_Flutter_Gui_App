#include "node/ros1/map_server_node.hpp"
#include "core/map_io.hpp"
#include "node/ros1/convert.hpp"
#include "common/logger/logger.h"

#include <stdexcept>

namespace nav2_map_server {

MapServerNode::MapServerNode() {}

MapServerNode::~MapServerNode() {}

bool MapServerNode::Init(const MapServerConfig& config) {
  yaml_filename_ = config.yaml_filename;
  core_.SetFrameId(config.frame_id);
  if (!core_.Initialize(config.tiles_http_port, config.tiles_output_dir)) {
    LOG_ERROR("Failed to initialize tiles map server");
    return false;
  }
  core_.Start();

  if (!yaml_filename_.empty()) {
    if (core_.LoadMapFromYaml(yaml_filename_) != LOAD_MAP_SUCCESS) {
      LOG_ERROR("Failed to load map: " << yaml_filename_);
      return false;
    }
  }

  map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>(config.pub_map_topic, 1, true);
  get_map_service_ = nh_.advertiseService("static_map", &MapServerNode::GetMapCallback, this);

  if (core_.IsMapAvailable()) {
    nav_msgs::OccupancyGrid msg;
    Convert(core_.GetMapData(), msg, config.frame_id);
    map_pub_.publish(msg);
  }

  return true;
}

void MapServerNode::Run() {
  ros::spin();
}

void MapServerNode::Shutdown() {
  core_.Shutdown();
  ros::shutdown();
}

void MapServerNode::GetMapCallback(nav_msgs::GetMap::Request&, nav_msgs::GetMap::Response& res) {
  Convert(core_.GetMapData(), res.map, core_.GetFrameId());
}

}  // namespace nav2_map_server
