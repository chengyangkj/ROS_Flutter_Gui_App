#pragma once

#include <cmath>

#include "core/occupancy_grid.hpp"
#include "nav_msgs/OccupancyGrid.h"
#include "ros/ros.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

namespace nav2_map_server {

inline void Convert(const OccupancyGridData& from, nav_msgs::OccupancyGrid& to,
    const std::string& frame_id) {
  to.info.width = from.width;
  to.info.height = from.height;
  to.info.resolution = from.resolution;
  to.info.origin.position.x = from.origin_x;
  to.info.origin.position.y = from.origin_y;
  to.info.origin.position.z = 0.0;
  tf2::Quaternion q;
  q.setRPY(0, 0, from.origin_yaw);
  to.info.origin.orientation = tf2::toMsg(q);
  to.info.map_load_time = ros::Time::now();
  to.header.frame_id = frame_id;
  to.header.stamp = ros::Time::now();
  to.data = from.data;
}

inline void Convert(const nav_msgs::OccupancyGrid& from, OccupancyGridData& to) {
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

}  // namespace nav2_map_server
