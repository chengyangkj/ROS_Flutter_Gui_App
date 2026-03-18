#include "node/ros1/map_server_node.hpp"
#include "core/map_io.hpp"
#include "node/ros1/convert.hpp"
#include "common/logger/logger.h"

#include <stdexcept>

namespace nav2_map_server {

MapServerNode::MapServerNode() {}

MapServerNode::~MapServerNode() {}

void MapServerNode::PublishMapUpdate() {
  if (map_manager_ && map_manager_->IsMapAvailable()) {
    nav_msgs::OccupancyGrid msg;
    Convert(map_manager_->GetMapData(), msg, map_manager_->GetFrameId());
    map_pub_.publish(msg);
  }
}

bool MapServerNode::Init(const MapServerConfig& config, MapManager* map_manager) {
  map_manager_ = map_manager;
  config_ = config;
  map_manager_->SetOnMapUpdateCallback([this]() { PublishMapUpdate(); });

  map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>(config.pub_map_topic, 1, true);
  get_map_service_ = nh_.advertiseService("static_map", &MapServerNode::GetMapCallback, this);

  PublishMapUpdate();
  return true;
}

void MapServerNode::Run() {
  ros::spin();
}

void MapServerNode::Shutdown() {
  ros::shutdown();
}

void MapServerNode::GetMapCallback(nav_msgs::GetMap::Request&, nav_msgs::GetMap::Response& res) {
  if (map_manager_) {
    Convert(map_manager_->GetMapData(), res.map, map_manager_->GetFrameId());
  }
}

}  // namespace nav2_map_server
