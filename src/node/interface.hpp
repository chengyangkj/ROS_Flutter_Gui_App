#pragma once

#include "node/gui_backend_config.hpp"

namespace ros_gui_backend {

class IRosGuiNode {
 public:
  virtual ~IRosGuiNode() = default;
  virtual bool Init(const GuiBackendConfig& config) = 0;
  virtual void Run() = 0;
  virtual void Shutdown() = 0;
};

}  // namespace ros_gui_backend
