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

bool RosGuiNode::Init(const GuiAppSettings& gui_app) {
  (void)gui_app;
  MapManager::Instance()->SetOnMapUpdateCallback([this]() { PublishMapUpdate(); });

  map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/map_manager/map", 1, true);
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

bool RosGuiNode::SetRobotStreamImageSubscription(
    const std::string& topic, bool subscribe, std::string* error_message) {
  (void)topic;
  (void)subscribe;
  (void)error_message;
  return true;
}

bool RosGuiNode::ReloadGuiStreams(const GuiAppSettings& settings) {
  (void)settings;
  return false;
}

bool RosGuiNode::PublishCmdVel(double vx, double vy, double vw) {
  (void)vx;
  (void)vy;
  (void)vw;
  return false;
}

bool RosGuiNode::PublishNavGoal(double x, double y, double roll, double pitch, double yaw) {
  (void)x;
  (void)y;
  (void)roll;
  (void)pitch;
  (void)yaw;
  return false;
}

bool RosGuiNode::PublishInitialPose(double x, double y, double roll, double pitch, double yaw) {
  (void)x;
  (void)y;
  (void)roll;
  (void)pitch;
  (void)yaw;
  return false;
}

bool RosGuiNode::PublishNavCancel() {
  return false;
}

bool RosGuiNode::LookupTransform(const std::string& target_frame, const std::string& source_frame,
    std::string* json_out, std::string* err) {
  (void)target_frame;
  (void)source_frame;
  (void)json_out;
  if (err) {
    *err = "ROS1 backend: transform API not implemented";
  }
  return false;
}

void RosGuiNode::GetMapCallback(nav_msgs::GetMap::Request&, nav_msgs::GetMap::Response& res) {
  MapManager* mm = MapManager::Instance();
  if (mm) {
    Convert(mm->GetMapData(), res.map, mm->GetFrameId());
  }
}

}  // namespace ros_gui_backend
