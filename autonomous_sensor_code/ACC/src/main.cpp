#include <Arduino.h>
#include <Wire.h>
#include "lis2de12.h"
#include "ble_service.h"
#include "power.h"

// ====== 用户可调参数 ======
static constexpr uint8_t  I2C_ADDR = LIS2DE12_I2C_ADDR; // 如 SA0 拉高改为 0x19（手册说明）:contentReference[oaicite:24]{index=24}
static constexpr uint32_t I2C_FREQ = 400000;            // 芯片支持 Fast-mode 400kHz :contentReference[oaicite:25]{index=25}
static constexpr uint8_t  FIFO_WATERMARK = 16;          // 每次攒16组样本再唤醒/上报
static constexpr uint32_t SLEEP_INTERVAL_MS = 200;      // 定时轮询（或接 INT1 更省）

// 打包：每个样本3字节（x,y,z，8bit），最多一次打包 32 组（96字节）
static uint8_t txBuf[3 * 32];

void setup() {
  Serial.begin(115200);
  delay(100);

  Power::init();

  Wire.begin();          // 默认 SDA/SCL 请按你的板子接线
  Wire.setClock(I2C_FREQ);

  if (!LIS2DE12::begin(Wire, I2C_ADDR)) {
    Serial.println("LIS2DE12 not found (WHO_AM_I!=0x33).");
    while (1) { delay(1000); }
  }
  Serial.println("LIS2DE12 OK.");

  // 低功耗配置：10Hz、±2g、BDU=1、FIFO Stream+WTM
  if (!LIS2DE12::configureLowPowerFIFO(FIFO_WATERMARK)) {
    Serial.println("Config error");
    while (1) { delay(1000); }
  }

  BLEAccel::begin("C3-LP-ACC");
}

void loop() {
  // 轮询 FIFO 溢水标志或定时
  uint8_t flags=0;
  uint8_t fss = LIS2DE12::fifoLevel(flags); // FSS=未读样本数（0~32）:contentReference[oaicite:26]{index=26}

  if (fss == 0) {
    // 没数据，轻睡一会
    Power::lightSleepFor((uint64_t)SLEEP_INTERVAL_MS * 1000ULL);
    return;
  }

  // 一次取最多 32 组，按 fss 取
  LIS2DE12::Sample8 tmp[32];
  size_t n = LIS2DE12::readFifoBurst(tmp, min<int>(fss, 32));
  if (n == 0) {
    // I2C 短故障，稍后重试
    delay(5);
    return;
  }

  // 简单打包：每样本三字节（x,y,z）
  size_t idx = 0;
  for (size_t i=0; i<n; ++i) {
    txBuf[idx++] = (uint8_t)tmp[i].x;
    txBuf[idx++] = (uint8_t)tmp[i].y;
    txBuf[idx++] = (uint8_t)tmp[i].z;
  }

  // 通过 BLE Notify 发送（注意 MTU；上面 setMTU(100)，一次发 99 字节以内较稳）
  // 分片发送
  size_t off = 0;
  const size_t CHUNK = 90; // 留一点余量
  while (off < idx) {
    size_t len = min(CHUNK, idx - off);
    BLEAccel::notifySamples(&txBuf[off], len);
    off += len;
    delay(2); // 让协议栈喘口气
  }

  // 轻睡到下一轮
  Power::lightSleepFor(50 * 1000); // 50ms 小憩；可直接等到 SLEEP_INTERVAL_MS
}
