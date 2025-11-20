#pragma once
#include <Arduino.h>

namespace Power {
  void init();
  // 轻睡一段时间（微秒）
  void lightSleepFor(uint64_t us);
}
