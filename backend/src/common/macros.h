#pragma once

#include <mutex>

#define DISABLE_COPY_AND_ASSIGN(class_name) \
  class_name(const class_name&) = delete; \
  class_name& operator=(const class_name&) = delete;

#define DEFINE_SINGLETON(class_name)                                        \
  public:                                                                   \
    static class_name* Instance(bool create_if_not_exist = true) {          \
        static class_name* instance = nullptr;                               \
        if (create_if_not_exist && instance == nullptr) {                   \
            static std::once_flag once;                                      \
            std::call_once(once, [&]() { instance = new class_name(); });   \
        }                                                                   \
        return instance;                                                    \
    }                                                                       \
    static void Clear() {                                                   \
        auto* inst = Instance(false);                                       \
        if (inst != nullptr) {                                              \
            delete inst;                                                    \
            inst = nullptr;                                                 \
        }                                                                   \
    }                                                                       \
  private:                                                                  \
    class_name();                                                           \
    DISABLE_COPY_AND_ASSIGN(class_name)
