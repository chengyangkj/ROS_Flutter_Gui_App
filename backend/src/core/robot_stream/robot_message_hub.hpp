#pragma once

#include <drogon/WebSocketConnection.h>
#include "robot_message.pb.h"

#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace ros_gui_backend {

class RobotMessageHub {
 public:
  static RobotMessageHub& Instance();

  void AddClient(const drogon::WebSocketConnectionPtr& conn);
  void RemoveClient(const drogon::WebSocketConnectionPtr& conn);
  void BroadcastBinary(const std::string& payload);
  void BroadcastRobotMsg(const ros_gui_backend::pb::RobotMessage& msg);

 private:
  RobotMessageHub() = default;

  std::mutex mu_;
  std::vector<std::weak_ptr<drogon::WebSocketConnection>> clients_;
};

}  // namespace ros_gui_backend
