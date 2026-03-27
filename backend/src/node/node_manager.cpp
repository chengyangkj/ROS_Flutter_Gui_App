#include "node/node_manager.hpp"

#include "common/config/config.hpp"

#if ROS_VERSION == 1
#include "node/ros1/node.hpp"
#else
#include "node/ros2/node.hpp"
#endif

namespace ros_gui_backend {

NodeManager::NodeManager() = default;

bool NodeManager::InitNode() {
  std::lock_guard<std::mutex> lk(mu_);
  if (node_) {
    return false;
  }
  auto n = std::make_shared<RosGuiNode>();
  if (!n->Init(Config::Instance()->GuiApp())) {
    return false;
  }
  node_ = std::move(n);
  return true;
}

void NodeManager::Reset() {
  std::shared_ptr<IRosGuiNode> n;
  {
    std::lock_guard<std::mutex> lk(mu_);
    n = std::move(node_);
  }
  if (n) {
    n->Shutdown();
  }
}

std::shared_ptr<IRosGuiNode> NodeManager::GetNode() const {
  std::lock_guard<std::mutex> lk(mu_);
  return node_;
}

bool NodeManager::SetRobotStreamImageSubscription(
    const std::string& topic, bool subscribe, std::string* error_message) {
  auto n = GetNode();
  if (!n) {
    if (error_message) {
      *error_message = "ros node not ready";
    }
    return false;
  }
  return n->SetRobotStreamImageSubscription(topic, subscribe, error_message);
}

}  // namespace ros_gui_backend
