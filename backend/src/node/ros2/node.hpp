#pragma once

#include "node/interface.hpp"
#include "common/config/config.hpp"
#include "core/map/map_manager.hpp"
#include "core/map/map_io.hpp"
#include "node/ros2/convert.hpp"

#include "action_msgs/msg/goal_status_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/empty.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "nav_msgs/srv/get_map.hpp"
#include "nav2_msgs/srv/load_map.hpp"
#include "nav2_msgs/srv/save_map.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"

#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

namespace ros_gui_backend {

class RosGuiNode : public rclcpp::Node, public IRosGuiNode {
 public:
  RosGuiNode();
  ~RosGuiNode() override;

  bool Init(const GuiAppSettings& gui_app) override;
  void Run() override;
  void Shutdown() override;
  bool SetRobotStreamImageSubscription(
      const std::string& topic, bool subscribe, std::string* error_message) override;
  bool ReloadGuiStreams(const GuiAppSettings& settings) override;
  bool PublishCmdVel(double vx, double vy, double vw) override;
  bool PublishNavGoal(double x, double y, double roll, double pitch, double yaw) override;
  bool PublishInitialPose(double x, double y, double roll, double pitch, double yaw) override;
  bool PublishNavCancel() override;
  bool LookupTransform(const std::string& target_frame, const std::string& source_frame,
      std::string* json_out, std::string* err) override;

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
  void RawOccMapUpdateCallback(const std::shared_ptr<OccupancyGridData> msg);
  bool SaveMapTopicToFile(const std::string& map_topic, const SaveParameters& save_parameters);

  void PublishMapUpdate();
  void SetupGuiStreamsLocked();
  void ClearGuiStreamsLocked();
  void PoseTimerTick();
  void OnLaser(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void OnLocalPath(const nav_msgs::msg::Path::SharedPtr msg);
  void OnGlobalPath(const nav_msgs::msg::Path::SharedPtr msg);
  void OnTracePath(const nav_msgs::msg::Path::SharedPtr msg);
  void OnOdom(const nav_msgs::msg::Odometry::SharedPtr msg);
  void OnBattery(const sensor_msgs::msg::BatteryState::SharedPtr msg);
  void OnFootprint(const geometry_msgs::msg::PolygonStamped::SharedPtr msg);
  void OnLocalCostmap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void OnGlobalCostmap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void OnPointCloud2(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void OnDiagnostic(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg);
  void OnGoalStatus(const action_msgs::msg::GoalStatusArray::SharedPtr msg);

  void OnCameraImage(const std::string& ros_topic, const sensor_msgs::msg::Image::SharedPtr msg);
  void OnCompressedImage(
      const std::string& ros_topic, const sensor_msgs::msg::CompressedImage::SharedPtr msg);

  rclcpp::Service<nav_msgs::srv::GetMap>::SharedPtr get_map_service_;
  rclcpp::Service<nav2_msgs::srv::LoadMap>::SharedPtr load_map_service_;
  rclcpp::Service<nav2_msgs::srv::SaveMap>::SharedPtr save_map_service_;
  rclcpp::Publisher<OccGridPubAdaptedType>::SharedPtr occ_pub_;
  rclcpp::Subscription<OccGridDataAdaptedType>::SharedPtr raw_occ_map_sub_;
  rclcpp::CallbackGroup::SharedPtr image_callback_group_;
  rclcpp::CallbackGroup::SharedPtr stream_callback_group_;
  std::unordered_map<std::string, rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr>
      image_subs_;
  std::unordered_map<std::string, rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr>
      compressed_image_subs_;

  GuiAppSettings gui_settings_;
  std::mutex image_mu_;
  std::mutex stream_mu_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::TimerBase::SharedPtr pose_timer_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr nav_goal_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr reloc_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr nav_cancel_pub_;

  std::vector<rclcpp::SubscriptionBase::SharedPtr> stream_subs_;

  std::chrono::steady_clock::time_point last_local_costmap_push_;
  std::chrono::steady_clock::time_point last_global_costmap_push_;
};

}  // namespace ros_gui_backend
