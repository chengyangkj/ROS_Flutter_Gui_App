#pragma once

#include "node/map_server_interface.hpp"
#include "core/map_server_core.hpp"

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>

namespace nav2_map_server {

class MapServerNode : public IMapServerNode {
public:
  MapServerNode();
  ~MapServerNode();

  bool Init(const MapServerConfig& config) override;
  void Run() override;
  void Shutdown() override;

private:
  void GetMapCallback(nav_msgs::GetMap::Request& req, nav_msgs::GetMap::Response& res);

  ros::NodeHandle nh_;
  ros::Publisher map_pub_;
  ros::ServiceServer get_map_service_;

  MapServerCore core_;
  std::string yaml_filename_;
};

}  // namespace nav2_map_server
