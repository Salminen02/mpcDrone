#pragma once
#include <atomic>

class IdGenerator {
public:
  /**
   * @brief Get the next unique ID.
   * Thread-safe ID generation. Each call returns counter++.
   * @return int The next unique ID
   */
  static int next() {
    static std::atomic<int> counter{0};
    return counter++;
  }
};