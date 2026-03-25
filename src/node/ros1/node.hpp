#pragma once

#include "node/interface.hpp"
#include "core/map/map_manager.hpp"

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>

namespace ros_gui_backend {

class RosGuiNode : public IRosGuiNode {
 public:
  RosGuiNode();
  ~RosGuiNode() override;

  bool Init(const GuiBackendConfig& config) override;
  void Run() override;
  void Shutdown() override;

 private:
  void GetMapCallback(nav_msgs::GetMap::Request& req, nav_msgs::GetMap::Response& res);

  ros::NodeHandle nh_;
  ros::Publisher map_pub_;
  ros::ServiceServer get_map_service_;

  GuiBackendConfig config_;

  void PublishMapUpdate();
};

}  // namespace ros_gui_backend
