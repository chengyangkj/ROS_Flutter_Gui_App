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
  void GetMapCallback(nav_msgs::GetMap::Request& req, nav_msgs::GetMap::Response& res);

  ros::NodeHandle nh_;
  ros::Publisher map_pub_;
  ros::ServiceServer get_map_service_;

  void PublishMapUpdate();
};

}  // namespace ros_gui_backend
