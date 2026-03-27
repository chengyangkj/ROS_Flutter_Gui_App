#pragma once

#include "app/app_config.hpp"

#include <memory>
#include <string>

namespace ros_gui_backend {

class WebServer;

class Application {
 public:
  Application();
  ~Application();

  bool Initialize(int argc, char** argv);
  int Start();
  void Stop();

 private:
  std::string ParseConfigYamlPathFromArgs(int argc, char** argv) const;
  bool LoadSettingsFromYaml(std::string* error_message);

  int argc_{0};
  char** argv_{nullptr};
  std::string config_yaml_path_;
  AppYamlSettings settings_;
  std::unique_ptr<WebServer> web_server_;
  bool initialized_{false};
  bool ros_runtime_inited_{false};
  bool stopped_{false};
};

}  // namespace ros_gui_backend
