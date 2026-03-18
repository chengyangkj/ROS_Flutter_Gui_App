#include "node/ros2/map_server_node.hpp"
#include "core/map_io.hpp"
#include "node/ros2/convert.hpp"
#include "common/logger/logger.h"

#include <boost/filesystem.hpp>
#include <memory>
#include <stdexcept>
#include <string>

namespace fs = boost::filesystem;

using namespace std::placeholders;

namespace nav2_map_server {

MapServerNode::MapServerNode(const MapServerConfig& config)
    : rclcpp::Node("map_manager"), config_(config) {}

MapServerNode::~MapServerNode() {}

void MapServerNode::PublishMapUpdate() {
  if (map_manager_ && map_manager_->IsMapAvailable() && occ_pub_ && topo_map_pub_) {
    occ_pub_->publish(OccGridPubMsg{map_manager_->GetMapData(), map_manager_->GetFrameId(), now()});
    topo_map_pub_->publish(map_manager_->GetTopoMap());
  }
}

bool MapServerNode::Init(const MapServerConfig& config, MapManager* map_manager) {
  map_manager_ = map_manager;
  config_ = config;
  map_manager_->SetOnMapUpdateCallback([this]() { PublishMapUpdate(); });

  const std::string& pub_topic = config_.pub_map_topic;
  const std::string& sub_topic = config_.sub_map_topic;

  const std::string prefix = get_name() + std::string("/");
  get_map_service_ = create_service<nav_msgs::srv::GetMap>(
      prefix + "map", std::bind(&MapServerNode::GetMapCallback, this, _1, _2, _3));
  load_map_service_ = create_service<nav2_msgs::srv::LoadMap>(
      prefix + "load_map", std::bind(&MapServerNode::LoadMapCallback, this, _1, _2, _3));
  save_map_service_ = create_service<nav2_msgs::srv::SaveMap>(
      prefix + "save_map", std::bind(&MapServerNode::SaveMapCallback, this, _1, _2, _3));

  occ_pub_ = create_publisher<OccGridPubAdaptedType>(
      pub_topic, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  topo_map_pub_ = create_publisher<TopoMapAdaptedType>(
      sub_topic + "/topology", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  raw_occ_map_sub_ = create_subscription<OccGridDataAdaptedType>(
      sub_topic,
      rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
      std::bind(&MapServerNode::RawOccMapUpdateCallback, this, _1));

  PublishMapUpdate();
  return true;
}

void MapServerNode::Run() {
  rclcpp::spin(shared_from_this());
}

void MapServerNode::Shutdown() {
  rclcpp::shutdown();
}

bool MapServerNode::LoadMapResponseFromYaml(const std::string& yaml_file,
    std::shared_ptr<nav2_msgs::srv::LoadMap::Response> response) {
  if (!map_manager_) return false;
  LOAD_MAP_STATUS status = map_manager_->LoadMapFromYaml(yaml_file);
  if (status == MAP_DOES_NOT_EXIST) {
    response->result = nav2_msgs::srv::LoadMap::Response::RESULT_MAP_DOES_NOT_EXIST;
    return false;
  }
  if (status == INVALID_MAP_METADATA) {
    response->result = nav2_msgs::srv::LoadMap::Response::RESULT_INVALID_MAP_METADATA;
    return false;
  }
  if (status == INVALID_MAP_DATA) {
    response->result = nav2_msgs::srv::LoadMap::Response::RESULT_INVALID_MAP_DATA;
    return false;
  }
  FillOccGridMsg(map_manager_->GetMapData(), response->map, map_manager_->GetFrameId(), now());
  response->result = nav2_msgs::srv::LoadMap::Response::RESULT_SUCCESS;
  return true;
}

void MapServerNode::GetMapCallback(
    const std::shared_ptr<rmw_request_id_t>,
    const std::shared_ptr<nav_msgs::srv::GetMap::Request>,
    std::shared_ptr<nav_msgs::srv::GetMap::Response> response) {
  if (map_manager_) {
    FillOccGridMsg(map_manager_->GetMapData(), response->map, map_manager_->GetFrameId(), now());
  }
}

void MapServerNode::LoadMapCallback(
    const std::shared_ptr<rmw_request_id_t>,
    const std::shared_ptr<nav2_msgs::srv::LoadMap::Request> request,
    std::shared_ptr<nav2_msgs::srv::LoadMap::Response> response) {
  if (LoadMapResponseFromYaml(request->map_url, response) && map_manager_) {
    occ_pub_->publish(OccGridPubMsg{map_manager_->GetMapData(), map_manager_->GetFrameId(), now()});
    topo_map_pub_->publish(map_manager_->GetTopoMap());
  }
}

void MapServerNode::SaveMapCallback(
    const std::shared_ptr<rmw_request_id_t>,
    const std::shared_ptr<nav2_msgs::srv::SaveMap::Request> request,
    std::shared_ptr<nav2_msgs::srv::SaveMap::Response> response) {
  SaveParameters save_parameters;
  save_parameters.map_file_name = request->map_url;
  save_parameters.image_format = request->image_format;
  save_parameters.free_thresh = request->free_thresh;
  save_parameters.occupied_thresh = request->occupied_thresh;
  try {
    save_parameters.mode = map_mode_from_string(request->map_mode);
  } catch (std::invalid_argument&) {
    save_parameters.mode = MapMode::Trinary;
  }
  response->result = SaveMapTopicToFile(request->map_topic, save_parameters);
}

void MapServerNode::RawOccMapUpdateCallback(const std::shared_ptr<OccupancyGridData> msg) {
  if (map_manager_) map_manager_->UpdateMap(*msg);
}

bool MapServerNode::SaveMapTopicToFile(const std::string& map_topic,
    const SaveParameters& save_parameters) {
  std::string topic = map_topic.empty() ? "map" : map_topic;
  SaveParameters params = save_parameters;
  if (params.free_thresh == 0.0) params.free_thresh = config_.free_thresh_default;
  if (params.occupied_thresh == 0.0) params.occupied_thresh = config_.occupied_thresh_default;

  std::promise<std::shared_ptr<OccupancyGridData>> prom;
  auto future_result = prom.get_future();
  auto map_cb = [&prom](std::shared_ptr<OccupancyGridData> msg) { prom.set_value(msg); };

  rclcpp::QoS map_qos(10);
  if (config_.map_subscribe_transient_local) {
    map_qos.transient_local().reliable().keep_last(1);
  }

  auto callback_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
  rclcpp::SubscriptionOptions opt;
  opt.callback_group = callback_group;
  auto map_sub = create_subscription<OccGridDataAdaptedType>(topic, map_qos, map_cb, opt);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_callback_group(callback_group, get_node_base_interface());
  rclcpp::Duration save_timeout(
      rclcpp::Duration::from_seconds(config_.save_map_timeout));
  auto timeout = save_timeout.to_chrono<std::chrono::nanoseconds>();
  if (executor.spin_until_future_complete(future_result, timeout) != rclcpp::FutureReturnCode::SUCCESS) {
    LOG_ERROR("Failed to receive map from topic");
    return false;
  }

  map_sub.reset();
  auto map_data = future_result.get();
  return saveMapToFile(*map_data, params);
}

}  // namespace nav2_map_server
