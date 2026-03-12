#pragma once

#include "node/map_server_interface.hpp"
#include "core/map_server_core.hpp"
#include "core/map_io.hpp"
#include "core/topology_map.hpp"
#include "node/ros2/convert.hpp"

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/srv/get_map.hpp"
#include "nav2_msgs/srv/load_map.hpp"
#include "nav2_msgs/srv/save_map.hpp"
#include "topology_msgs/msg/topology_map.hpp"

namespace nav2_map_server {

class MapServerNode : public rclcpp::Node, public IMapServerNode {
public:
  explicit MapServerNode(const MapServerConfig& config);
  ~MapServerNode();

  void Run() override;
  void Shutdown() override;

private:
  void Setup();

  bool LoadMapResponseFromYaml(const std::string& yaml_file,
      std::shared_ptr<nav2_msgs::srv::LoadMap::Response> response);
  void GetMapCallback(const std::shared_ptr<rmw_request_id_t>,
      const std::shared_ptr<nav_msgs::srv::GetMap::Request>,
      std::shared_ptr<nav_msgs::srv::GetMap::Response>);
  void LoadMapCallback(const std::shared_ptr<rmw_request_id_t>,
      const std::shared_ptr<nav2_msgs::srv::LoadMap::Request>,
      std::shared_ptr<nav2_msgs::srv::LoadMap::Response>);
  void SaveMapCallback(const std::shared_ptr<rmw_request_id_t>,
      const std::shared_ptr<nav2_msgs::srv::SaveMap::Request>,
      std::shared_ptr<nav2_msgs::srv::SaveMap::Response>);
  void TopoMapUpdateCallback(const std::shared_ptr<TopologyMap>);
  void OccMapUpdateCallback(const std::shared_ptr<OccupancyGridData>);
  void RawOccMapUpdateCallback(const std::shared_ptr<OccupancyGridData>);
  bool SaveMapTopicToFile(const std::string& map_topic, const SaveParameters& save_parameters);

  rclcpp::Service<nav_msgs::srv::GetMap>::SharedPtr get_map_service_;
  rclcpp::Service<nav2_msgs::srv::LoadMap>::SharedPtr load_map_service_;
  rclcpp::Service<nav2_msgs::srv::SaveMap>::SharedPtr save_map_service_;
  rclcpp::Publisher<OccGridPubAdaptedType>::SharedPtr occ_pub_;
  rclcpp::Publisher<TopoMapAdaptedType>::SharedPtr topo_map_pub_;
  rclcpp::Subscription<TopoMapAdaptedType>::SharedPtr topo_map_sub_;
  rclcpp::Subscription<OccGridDataAdaptedType>::SharedPtr occ_map_update_sub_;
  rclcpp::Subscription<OccGridDataAdaptedType>::SharedPtr raw_occ_map_sub_;

  MapServerCore core_;
  MapServerConfig config_;
  std::string yaml_filename_;
};

}  // namespace nav2_map_server
