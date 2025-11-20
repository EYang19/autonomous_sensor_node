#pragma once
#include <Arduino.h>

namespace Power {
  void init();
  void lightSleepFor(uint64_t us);
}
