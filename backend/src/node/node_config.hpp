#pragma once

#include <string>

namespace ros_gui_backend {

struct NodeConfig {
  std::string map_frame{"map"};
  std::string base_link_frame{"base_link"};
  std::string laser_topic{"scan"};
  std::string pointcloud_topic{"points"};
  std::string global_path_topic{"/plan"};
  std::string local_path_topic{"/local_plan"};
  std::string trace_path_topic{"/transformed_global_plan"};
  std::string odom_topic{"/wheel/odometry"};
  std::string battery_topic{"/battery_status"};
  std::string robot_footprint_topic{"/local_costmap/published_footprint"};
  std::string local_costmap_topic{"/local_costmap/costmap"};
  std::string global_costmap_topic{"/global_costmap/costmap"};
  std::string diagnostic_topic{"/diagnostics"};
  std::string topology_topic{"/map/topology"};
  std::string topology_json_topic{};
  std::string nav_to_pose_status_topic{"navigate_to_pose/_action/status"};
  std::string nav_through_poses_status_topic{"navigate_through_poses/_action/status"};
  std::string reloc_topic{"/initialpose"};
  std::string nav_goal_topic{"/goal_pose"};
  std::string speed_ctrl_topic{"/cmd_vel"};
  std::string topology_publish_topic{"/map/topology/update"};
};

}  // namespace ros_gui_backend
