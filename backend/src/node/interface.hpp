#pragma once

#include "common/config/config.hpp"

#include <string>

namespace ros_gui_backend {

class IRosGuiNode {
 public:
  virtual ~IRosGuiNode() = default;
  virtual bool Init(const GuiAppSettings& gui_app) = 0;
  virtual void Run() = 0;
  virtual void Shutdown() = 0;
  virtual bool SetRobotStreamImageSubscription(
      const std::string& topic, bool subscribe, std::string* error_message) = 0;
  virtual bool ReloadGuiStreams(const GuiAppSettings& settings) = 0;
  virtual bool PublishCmdVel(double vx, double vy, double vw) = 0;
  virtual bool PublishNavGoal(double x, double y, double roll, double pitch, double yaw) = 0;
  virtual bool PublishInitialPose(double x, double y, double roll, double pitch, double yaw) = 0;
  virtual bool PublishNavCancel() = 0;
  virtual bool LookupTransform(const std::string& target_frame, const std::string& source_frame,
      std::string* json_out, std::string* err) = 0;
};

}  // namespace ros_gui_backend
