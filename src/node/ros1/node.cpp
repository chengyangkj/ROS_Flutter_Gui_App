#include "node/ros1/node.hpp"
#include "core/map/map_io.hpp"
#include "node/ros1/convert.hpp"
#include "common/logger/logger.h"

#include <stdexcept>

namespace ros_gui_backend {

RosGuiNode::RosGuiNode() {}

RosGuiNode::~RosGuiNode() {}

void RosGuiNode::PublishMapUpdate() {
  MapManager* mm = MapManager::Instance();
  if (mm && mm->IsMapAvailable()) {
    nav_msgs::OccupancyGrid msg;
    Convert(mm->GetMapData(), msg, mm->GetFrameId());
    map_pub_.publish(msg);
  }
}

bool RosGuiNode::Init(const GuiBackendConfig& config) {
  config_ = config;
  MapManager::Instance()->SetOnMapUpdateCallback([this]() { PublishMapUpdate(); });

  map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>(config.pub_map_topic, 1, true);
  get_map_service_ = nh_.advertiseService("static_map", &RosGuiNode::GetMapCallback, this);

  PublishMapUpdate();
  return true;
}

void RosGuiNode::Run() {
  ros::spin();
}

void RosGuiNode::Shutdown() {
  ros::shutdown();
}

void RosGuiNode::GetMapCallback(nav_msgs::GetMap::Request&, nav_msgs::GetMap::Response& res) {
  MapManager* mm = MapManager::Instance();
  if (mm) {
    Convert(mm->GetMapData(), res.map, mm->GetFrameId());
  }
}

}  // namespace ros_gui_backend
