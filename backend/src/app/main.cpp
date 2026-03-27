#include "app/application.hpp"
#include "common/logger/logger.h"

int main(int argc, char** argv) {
  ros_gui_backend::Application app;
  if (!app.Initialize(argc, argv)) {
    return -1;
  }
  return app.Start();
}
