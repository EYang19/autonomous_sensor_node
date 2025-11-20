#pragma once
#include <Arduino.h>
#include <NimBLEDevice.h>

// 自定义服务/特征 UUID（随意生成或固定）
#define UUID_SVC_ACCEL   "8b37e000-6d6a-4a90-903b-22f43a1e2001"
#define UUID_CHR_SAMPLES "8b37e001-6d6a-4a90-903b-22f43a1e2001"

namespace BLEAccel {

void begin(const char* deviceName = "C3-LP-ACC");
void notifySamples(const uint8_t* payload, size_t len); // 最多 ~ MTU-3
} // namespace
