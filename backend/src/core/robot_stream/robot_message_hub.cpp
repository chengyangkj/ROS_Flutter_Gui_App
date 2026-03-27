#include "core/robot_stream/robot_message_hub.hpp"

#include <drogon/HttpAppFramework.h>

namespace ros_gui_backend {

RobotMessageHub& RobotMessageHub::Instance() {
  static RobotMessageHub inst;
  return inst;
}

void RobotMessageHub::AddClient(const drogon::WebSocketConnectionPtr& conn) {
  std::lock_guard<std::mutex> lock(mu_);
  clients_.push_back(conn);
}

void RobotMessageHub::RemoveClient(const drogon::WebSocketConnectionPtr& conn) {
  std::lock_guard<std::mutex> lock(mu_);
  for (auto it = clients_.begin(); it != clients_.end();) {
    auto sp = it->lock();
    if (!sp || sp == conn) {
      it = clients_.erase(it);
    } else {
      ++it;
    }
  }
}

void RobotMessageHub::BroadcastRobotMsg(const ros_gui_backend::pb::RobotMessage& msg) {
  std::string serialized;
  msg.SerializeToString(&serialized);
  BroadcastBinary(serialized);
}

void RobotMessageHub::BroadcastBinary(const std::string& payload) {
  std::vector<std::shared_ptr<drogon::WebSocketConnection>> snapshot;
  {
    std::lock_guard<std::mutex> lock(mu_);
    for (auto it = clients_.begin(); it != clients_.end();) {
      if (auto sp = it->lock()) {
        snapshot.push_back(sp);
        ++it;
      } else {
        it = clients_.erase(it);
      }
    }
  }
  auto* loop = drogon::app().getLoop();
  if (loop == nullptr) {
    return;
  }
  auto shared_payload = std::make_shared<std::string>(payload);
  for (const auto& c : snapshot) {
    std::weak_ptr<drogon::WebSocketConnection> weak = c;
    loop->runInLoop([weak, shared_payload]() {
      if (auto sp = weak.lock()) {
        sp->send(*shared_payload, drogon::WebSocketMessageType::Binary);
      }
    });
  }
}

}  // namespace ros_gui_backend
