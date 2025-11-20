#include "power.h"
#include <esp_sleep.h>

void Power::init() {
  // 关闭不必要的外设时钟，可按需扩展
}

void Power::lightSleepFor(uint64_t us) {
  esp_sleep_enable_timer_wakeup(us);
  esp_light_sleep_start(); // 轻睡，RAM/外设保持
}
