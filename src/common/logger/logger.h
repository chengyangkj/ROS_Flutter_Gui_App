#pragma once

#include <fmt/format.h>
#include <string>

#include "common/macros.h"
enum class LogLevel { INFO,
                      ERROR,
                      WARN };

#define LOGGER_INFO(...)                                                        \
  do {                                                                          \
    Logger::Instance()->Log(LogLevel::INFO, fmt::format(__VA_ARGS__), __FILE__, \
        __LINE__);                                                              \
  } while (0)

#define LOGGER_ERROR(...)                                                       \
  do {                                                                          \
    Logger::Instance()->Log(LogLevel::ERROR, fmt::format(__VA_ARGS__), __FILE__, \
        __LINE__);                                                              \
  } while (0)

#define LOGGER_WARN(...)                                                        \
  do {                                                                          \
    Logger::Instance()->Log(LogLevel::WARN, fmt::format(__VA_ARGS__), __FILE__, \
        __LINE__);                                                              \
  } while (0)

class Logger {
 public:
  ~Logger();
  void Log(LogLevel level, std::string message, const char* file = nullptr, int line = 0);

  DEFINE_SINGLETON(Logger)
};
