#pragma once

#include "common/macros.h"
#include "node/interface.hpp"

#include <memory>
#include <mutex>
#include <string>

namespace ros_gui_backend {

class NodeManager {
 public:
  bool InitNode();
  void Reset();

  std::shared_ptr<IRosGuiNode> GetNode() const;

  bool SetRobotStreamImageSubscription(
      const std::string& topic, bool subscribe, std::string* error_message);

 private:
  mutable std::mutex mu_;
  std::shared_ptr<IRosGuiNode> node_;

  DEFINE_SINGLETON(NodeManager)
};

}  // namespace ros_gui_backend
