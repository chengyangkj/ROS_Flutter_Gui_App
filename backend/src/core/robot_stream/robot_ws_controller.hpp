#pragma once

#include <drogon/WebSocketController.h>

namespace ros_gui_backend {

class RobotWsController : public drogon::WebSocketController<RobotWsController> {
 public:
  WS_PATH_LIST_BEGIN
  WS_PATH_ADD("/ws/robot");
  WS_PATH_LIST_END

  void handleNewMessage(const drogon::WebSocketConnectionPtr& conn, std::string&& message,
      const drogon::WebSocketMessageType& type) override;

  void handleNewConnection(
      const drogon::HttpRequestPtr& req, const drogon::WebSocketConnectionPtr& conn) override;

  void handleConnectionClosed(const drogon::WebSocketConnectionPtr& conn) override;
};



}  // namespace ros_gui_backend
