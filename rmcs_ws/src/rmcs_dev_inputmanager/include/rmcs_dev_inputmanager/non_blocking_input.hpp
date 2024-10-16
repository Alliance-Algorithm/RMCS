
#pragma once

#include "key_code.hpp"
#include <iostream>
#include <memory>
#include <sys/socket.h>
#include <unistd.h>
namespace test_utils::input_manager {

class InputManager {
public:
  InputManager(const InputManager &) = delete;
  InputManager &operator=(const InputManager &) = delete;
  static bool GetKey(KeyCode::KeyCodeEnum &&data) {
    return instance_->keycode_ == data;
  }

private:
  InputManager() : keycode_{} {};
  ~InputManager() = default;

  static bool stdinHasData() {
#if defined(__GNUG__) || defined(__GNUC__)
    struct timespec timeout {
      0l, 0l
    };

    fd_set fds{};
    FD_ZERO(&fds);
    FD_SET(0, &fds);

    return pselect(0 + 1, &fds, nullptr, nullptr, &timeout, nullptr) == 1;
#else
    // throw a compiler error
    static_assert(false, "Failed to detect a supported operating system!");
#endif
  }

  static std::unique_ptr<InputManager> instance_;
  KeyCode keycode_;
};
} // namespace test_utils::input_manager