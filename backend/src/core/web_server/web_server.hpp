#pragma once

#include <atomic>
#include <functional>
#include <memory>
#include <string>
#include <thread>

namespace ros_gui_backend {

struct WebServerConfig {
  int port{8080};
  std::string document_root;
};

class WebServer {
 public:
  WebServer();
  ~WebServer();

  static void SetSigintHook(std::function<void()> hook);
  static void ClearSigintHook();

  bool Start(const WebServerConfig& config);
  void Shutdown();

 private:
  void RunImpl(WebServerConfig config);

  std::unique_ptr<std::thread> thread_;
  std::atomic<bool> running_{false};
};

}  // namespace ros_gui_backend
