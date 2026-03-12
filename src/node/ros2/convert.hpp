#pragma once

#include <cmath>

#include "core/occupancy_grid.hpp"
#include "core/topology_map.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/type_adapter.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "topology_msgs/msg/topology_map.hpp"

namespace nav2_map_server {

struct OccGridPubMsg {
  const OccupancyGridData& data;
  std::string frame_id;
  rclcpp::Time stamp;
};

}  // namespace nav2_map_server

namespace rclcpp {

template <>
struct TypeAdapter<nav2_map_server::OccGridPubMsg, nav_msgs::msg::OccupancyGrid> {
  using is_specialized = std::true_type;
  using custom_type = nav2_map_server::OccGridPubMsg;
  using ros_message_type = nav_msgs::msg::OccupancyGrid;

  static void convert_to_ros_message(const custom_type& from, ros_message_type& to) {
    const auto& d = from.data;
    to.info.width = d.width;
    to.info.height = d.height;
    to.info.resolution = d.resolution;
    to.info.origin.position.x = d.origin_x;
    to.info.origin.position.y = d.origin_y;
    to.info.origin.position.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, d.origin_yaw);
    to.info.origin.orientation = tf2::toMsg(q);
    to.info.map_load_time = from.stamp;
    to.header.frame_id = from.frame_id;
    to.header.stamp = from.stamp;
    to.data = d.data;
  }
};

template <>
struct TypeAdapter<nav2_map_server::OccupancyGridData, nav_msgs::msg::OccupancyGrid> {
  using is_specialized = std::true_type;
  using custom_type = nav2_map_server::OccupancyGridData;
  using ros_message_type = nav_msgs::msg::OccupancyGrid;

  static void convert_to_ros_message(const custom_type& from, ros_message_type& to) {
    TypeAdapter<nav2_map_server::OccGridPubMsg, ros_message_type>::convert_to_ros_message(
        nav2_map_server::OccGridPubMsg{from, "", rclcpp::Time(0, 0)}, to);
  }

  static void convert_to_custom(const ros_message_type& from, custom_type& to) {
    to.width = from.info.width;
    to.height = from.info.height;
    to.resolution = from.info.resolution;
    to.origin_x = from.info.origin.position.x;
    to.origin_y = from.info.origin.position.y;
    double qw = from.info.origin.orientation.w;
    double qx = from.info.origin.orientation.x;
    double qy = from.info.origin.orientation.y;
    double qz = from.info.origin.orientation.z;
    to.origin_yaw = std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
    to.data = from.data;
  }
};

template <>
struct TypeAdapter<nav2_map_server::TopologyMap, topology_msgs::msg::TopologyMap> {
  using is_specialized = std::true_type;
  using custom_type = nav2_map_server::TopologyMap;
  using ros_message_type = topology_msgs::msg::TopologyMap;

  static void convert_to_ros_message(const custom_type& from, ros_message_type& to) {
    to.map_name = from.map_name;
    to.map_property.support_controllers = from.map_property.support_controllers;
    to.map_property.support_goal_checkers = from.map_property.support_goal_checkers;
    to.points.clear();
    for (const auto& point : from.points) {
      topology_msgs::msg::TopologyMapPointInfo point_info;
      point_info.x = point.x;
      point_info.y = point.y;
      point_info.theta = point.theta;
      point_info.name = point.name;
      switch (point.type) {
        case nav2_map_server::TopologyMapPointType::NavGoal:
          point_info.type = topology_msgs::msg::TopologyMapPointInfo::NAV_GOAL;
          break;
        case nav2_map_server::TopologyMapPointType::ChargingStation:
          point_info.type = topology_msgs::msg::TopologyMapPointInfo::CHARGE_STATION;
          break;
      }
      to.points.push_back(point_info);
    }
    to.routes.clear();
    for (const auto& route_pair : from.routes) {
      for (const auto& dest_pair : route_pair.second) {
        topology_msgs::msg::RouteConnection route_connection;
        route_connection.from_point = route_pair.first;
        route_connection.to_point = dest_pair.first;
        route_connection.route_info.controller = dest_pair.second.controller;
        route_connection.route_info.speed_limit = dest_pair.second.speed_limit;
        route_connection.route_info.goal_checker = dest_pair.second.goal_checker;
        to.routes.push_back(route_connection);
      }
    }
  }

  static void convert_to_custom(const ros_message_type& from, custom_type& to) {
    to.map_name = from.map_name;
    to.map_property.support_controllers = from.map_property.support_controllers;
    to.map_property.support_goal_checkers = from.map_property.support_goal_checkers;
    to.points.clear();
    for (const auto& point : from.points) {
      nav2_map_server::TopologyMap::PointInfo point_info;
      point_info.x = point.x;
      point_info.y = point.y;
      point_info.theta = point.theta;
      point_info.name = point.name;
      switch (point.type) {
        case topology_msgs::msg::TopologyMapPointInfo::NAV_GOAL:
          point_info.type = nav2_map_server::TopologyMapPointType::NavGoal;
          break;
        case topology_msgs::msg::TopologyMapPointInfo::CHARGE_STATION:
          point_info.type = nav2_map_server::TopologyMapPointType::ChargingStation;
          break;
        default:
          point_info.type = nav2_map_server::TopologyMapPointType::NavGoal;
          break;
      }
      to.points.push_back(point_info);
    }
    to.routes.clear();
    for (const auto& route : from.routes) {
      nav2_map_server::TopologyMap::RouteInfo route_info;
      route_info.controller = route.route_info.controller;
      route_info.speed_limit = route.route_info.speed_limit;
      route_info.goal_checker = route.route_info.goal_checker;
      to.routes[route.from_point][route.to_point] = route_info;
    }
  }
};

}  // namespace rclcpp

namespace nav2_map_server {

using OccGridPubAdaptedType = rclcpp::TypeAdapter<OccGridPubMsg, nav_msgs::msg::OccupancyGrid>;
using OccGridDataAdaptedType = rclcpp::TypeAdapter<OccupancyGridData, nav_msgs::msg::OccupancyGrid>;
using TopoMapAdaptedType = rclcpp::TypeAdapter<TopologyMap, topology_msgs::msg::TopologyMap>;

inline void FillOccGridMsg(const OccupancyGridData& data,
    nav_msgs::msg::OccupancyGrid& msg,
    const std::string& frame_id, const rclcpp::Time& stamp) {
  rclcpp::TypeAdapter<OccGridPubMsg, nav_msgs::msg::OccupancyGrid>::convert_to_ros_message(
      OccGridPubMsg{data, frame_id, stamp}, msg);
}

}  // namespace nav2_map_server
