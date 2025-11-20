#include "power.h"
#include <esp_sleep.h>

void Power::init() {
  // 可在此关闭不必要外设时钟/调节电源域
}

void Power::lightSleepFor(uint64_t us) {
  esp_sleep_enable_timer_wakeup(us);
  esp_light_sleep_start();
}
