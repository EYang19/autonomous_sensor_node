#include <Arduino.h>
#include <Wire.h>
#include "max30102.h"
#include "ble_service.h"
#include "power.h"

// ==== 用户可调 ====
// HR 低功耗：IR-only, 100 sps, LED 电流（0x24 ≈ 中低电流，按需下调以省电）:contentReference[oaicite:38]{index=38}
static constexpr bool     USE_SPO2 = false;     // true -> 红+IR；false -> 仅 IR
static constexpr uint32_t I2C_FREQ = 400000;    // MAX30102 支持 400 kHz :contentReference[oaicite:39]{index=39}
static constexpr uint8_t  FIFO_EMPTY_SLOTS = 16;// A_FULL 在 FIFO 剩余 16 空位时触发（32 深度）:contentReference[oaicite:40]{index=40}
static constexpr uint32_t POLL_INTERVAL_MS = 100; // 无中断时的轻睡轮询周期

// BLE 分片参数
static constexpr size_t CHUNK = 90; // MTU 100，留余量

// 发送缓存：SPO2 每样本6字节，FIFO最多32样本 -> 192B；HR 每样本3字节 -> 96B
static uint8_t txBuf[192];

void setup() {
  Serial.begin(115200);
  delay(50);

  Power::init();

  Wire.begin();
  Wire.setClock(I2C_FREQ);

  if (!MAX30102::begin(Wire, MAX30102_ADDR)) {
    Serial.println("MAX30102 not found (PART_ID != 0x15).");
    while (1) delay(1000);
  }

  bool ok=false;
  if (USE_SPO2) {
    ok = MAX30102::configureLowPowerSPO2(100, 0x18, 0x24, 8);
  } else {
    ok = MAX30102::configureLowPowerHR(100, 0x24, FIFO_EMPTY_SLOTS);
  }
  if (!ok) {
    Serial.println("Config failed");
    while (1) delay(1000);
  }

  BLEPPG::begin("C3-LP-PPG");
  Serial.println("PPG ready.");
}

void loop() {
  // 查看 FIFO 可读样本数（0~32）
  uint8_t avail = MAX30102::availableSamples();
  if (avail == 0) {
    Power::lightSleepFor(POLL_INTERVAL_MS * 1000ULL);
    return;
  }

  size_t idx = 0;
  if (USE_SPO2) {
    MAX30102::SampleSPO2 buf[32];
    size_t n = MAX30102::readFIFO_SPO2(buf, min<int>(avail, 32));
    // 打包：每样本6字节（RED, IR 24-bit 原样）
    for (size_t i=0; i<n; ++i) {
      txBuf[idx++] = (uint8_t)(buf[i].red24 >> 16);
      txBuf[idx++] = (uint8_t)(buf[i].red24 >> 8);
      txBuf[idx++] = (uint8_t)(buf[i].red24);
      txBuf[idx++] = (uint8_t)(buf[i].ir24 >> 16);
      txBuf[idx++] = (uint8_t)(buf[i].ir24 >> 8);
      txBuf[idx++] = (uint8_t)(buf[i].ir24);
    }
  } else {
    MAX30102::SampleHR buf[32];
    size_t n = MAX30102::readFIFO_HR(buf, min<int>(avail, 32));
    for (size_t i=0; i<n; ++i) {
      txBuf[idx++] = (uint8_t)(buf[i].ir24 >> 16);
      txBuf[idx++] = (uint8_t)(buf[i].ir24 >> 8);
      txBuf[idx++] = (uint8_t)(buf[i].ir24);
    }
  }

  // BLE 分片发送
  size_t off = 0;
  while (off < idx) {
    size_t len = min(CHUNK, idx - off);
    BLEPPG::notifySamples(&txBuf[off], len);
    off += len;
    delay(2);
  }

  // 小憩；若你把 INT 接到某 GPIO，可改为中断驱动+更长睡眠
  Power::lightSleepFor(50 * 1000);
}
