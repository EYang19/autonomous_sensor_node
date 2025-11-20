#pragma once
#include <Arduino.h>
#include <NimBLEDevice.h>

#define UUID_SVC_PPG    "fae6b800-1a5f-4c6b-8a28-2a42b2d60001"
#define UUID_CHR_PPG    "fae6b801-1a5f-4c6b-8a28-2a42b2d60001"

namespace BLEPPG {
  void begin(const char* name = "C3-LP-PPG");
  void notifySamples(const uint8_t* payload, size_t len); // 兼容不同 NimBLE 版本
}
