#pragma once

#include <cmath>

#include "core/map/occupancy_grid.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/type_adapter.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace ros_gui_backend {

struct OccGridPubMsg {
  const OccupancyGridData& data;
  std::string frame_id;
  rclcpp::Time stamp;
};

}  // namespace ros_gui_backend

namespace rclcpp {

template <>
struct TypeAdapter<ros_gui_backend::OccGridPubMsg, nav_msgs::msg::OccupancyGrid> {
  using is_specialized = std::true_type;
  using custom_type = ros_gui_backend::OccGridPubMsg;
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
struct TypeAdapter<ros_gui_backend::OccupancyGridData, nav_msgs::msg::OccupancyGrid> {
  using is_specialized = std::true_type;
  using custom_type = ros_gui_backend::OccupancyGridData;
  using ros_message_type = nav_msgs::msg::OccupancyGrid;

  static void convert_to_ros_message(const custom_type& from, ros_message_type& to) {
    TypeAdapter<ros_gui_backend::OccGridPubMsg, ros_message_type>::convert_to_ros_message(
        ros_gui_backend::OccGridPubMsg{from, "", rclcpp::Time(0, 0)}, to);
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

}  // namespace rclcpp

namespace ros_gui_backend {

using OccGridPubAdaptedType = rclcpp::TypeAdapter<OccGridPubMsg, nav_msgs::msg::OccupancyGrid>;
using OccGridDataAdaptedType = rclcpp::TypeAdapter<OccupancyGridData, nav_msgs::msg::OccupancyGrid>;

inline void FillOccGridMsg(const OccupancyGridData& data,
    nav_msgs::msg::OccupancyGrid& msg,
    const std::string& frame_id, const rclcpp::Time& stamp) {
  rclcpp::TypeAdapter<OccGridPubMsg, nav_msgs::msg::OccupancyGrid>::convert_to_ros_message(
      OccGridPubMsg{data, frame_id, stamp}, msg);
}

}  // namespace ros_gui_backend
