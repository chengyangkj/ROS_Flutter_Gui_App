#include "core/robot_stream/robot_ws_controller.hpp"

#include "core/robot_stream/robot_message_hub.hpp"
#include "common/logger/logger.h"

#include <drogon/WebSocketConnection.h>

namespace ros_gui_backend {

void RobotWsController::handleNewMessage(const drogon::WebSocketConnectionPtr&,
    std::string&& message, const drogon::WebSocketMessageType& type) {
  (void)message;
  if (type != drogon::WebSocketMessageType::Binary) {
    return;
  }
}

void RobotWsController::handleNewConnection(
    const drogon::HttpRequestPtr&, const drogon::WebSocketConnectionPtr& conn) {
  LOGGER_INFO("robot ws connected: {}", conn->peerAddr().toIpPort());
  RobotMessageHub::Instance().AddClient(conn);
}

void RobotWsController::handleConnectionClosed(const drogon::WebSocketConnectionPtr& conn) {
  LOGGER_INFO("robot ws disconnected: {}", conn->peerAddr().toIpPort());
  RobotMessageHub::Instance().RemoveClient(conn);
}

}  // namespace ros_gui_backend
