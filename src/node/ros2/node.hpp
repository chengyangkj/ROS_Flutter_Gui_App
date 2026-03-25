#pragma once

#include "node/interface.hpp"
#include "core/map/map_manager.hpp"
#include "core/map/map_io.hpp"
#include "node/ros2/convert.hpp"

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/srv/get_map.hpp"
#include "nav2_msgs/srv/load_map.hpp"
#include "nav2_msgs/srv/save_map.hpp"

namespace ros_gui_backend {

class RosGuiNode : public rclcpp::Node, public IRosGuiNode {
 public:
  explicit RosGuiNode(const GuiBackendConfig& config);
  ~RosGuiNode() override;

  bool Init(const GuiBackendConfig& config) override;
  void Run() override;
  void Shutdown() override;

 private:
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
  void RawOccMapUpdateCallback(const std::shared_ptr<OccupancyGridData>);
  bool SaveMapTopicToFile(const std::string& map_topic, const SaveParameters& save_parameters);

  rclcpp::Service<nav_msgs::srv::GetMap>::SharedPtr get_map_service_;
  rclcpp::Service<nav2_msgs::srv::LoadMap>::SharedPtr load_map_service_;
  rclcpp::Service<nav2_msgs::srv::SaveMap>::SharedPtr save_map_service_;
  rclcpp::Publisher<OccGridPubAdaptedType>::SharedPtr occ_pub_;
  rclcpp::Subscription<OccGridDataAdaptedType>::SharedPtr raw_occ_map_sub_;

  GuiBackendConfig config_;

  void PublishMapUpdate();
};

}  // namespace ros_gui_backend
