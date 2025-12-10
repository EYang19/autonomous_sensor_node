#include <Arduino.h>
#include <Wire.h>
#include <math.h>

// ==== 新增：BLE 相关头文件 ====
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

// ===== I2C 引脚：GPIO8=SCL, GPIO10=SDA =====
static const int I2C_SDA = 10;
static const int I2C_SCL = 3;
static const uint32_t I2C_FREQ = 400000;

static const uint8_t MAX30102_ADDR = 0x57;

// Registers
static const uint8_t REG_INTR_STATUS_1 = 0x00;
static const uint8_t REG_INTR_STATUS_2 = 0x01;
static const uint8_t REG_INTR_ENABLE_1 = 0x02;
static const uint8_t REG_INTR_ENABLE_2 = 0x03;
static const uint8_t REG_FIFO_WR_PTR   = 0x04;
static const uint8_t REG_OVF_COUNTER   = 0x05;
static const uint8_t REG_FIFO_RD_PTR   = 0x06;
static const uint8_t REG_FIFO_DATA     = 0x07;
static const uint8_t REG_FIFO_CONFIG   = 0x08;
static const uint8_t REG_MODE_CONFIG   = 0x09;
static const uint8_t REG_SPO2_CONFIG   = 0x0A;
static const uint8_t REG_LED1_PA       = 0x0C; // RED
static const uint8_t REG_LED2_PA       = 0x0D; // IR
static const uint8_t REG_MULTI_LED_CTRL1 = 0x11;
static const uint8_t REG_MULTI_LED_CTRL2 = 0x12;
static const uint8_t REG_REV_ID        = 0xFE;
static const uint8_t REG_PART_ID       = 0xFF;

// ==== 新增：BLE UUID 与名称（可按需改自己的 UUID） ====
#define SERVICE_UUID        "d3aa6a66-a623-4c76-80a5-a66004acf0bf"
#define CHARACTERISTIC_UUID "2df03c7d-8ac5-493e-9d88-ac467aed4ad0"
#define BLE_SERVER_NAME     "ppg"

// ==== 新增：BLE 全局对象 ====
BLEServer*        pServer        = nullptr;
BLECharacteristic* pCharacteristic = nullptr;
bool deviceConnected = false;

// ==== 新增：BLE 回调 ====
class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) override {
    deviceConnected = true;
    Serial.println("[BLE] Central connected");
  }
  void onDisconnect(BLEServer* pServer) override {
    deviceConnected = false;
    Serial.println("[BLE] Central disconnected, restart advertising");
    pServer->startAdvertising();
  }
};

// --- I2C helpers ---
static bool i2cWrite8(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MAX30102_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return (Wire.endTransmission() == 0);
}

static bool i2cRead8(uint8_t reg, uint8_t &val) {
  Wire.beginTransmission(MAX30102_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)MAX30102_ADDR, 1) != 1) return false;
  val = Wire.read();
  return true;
}

static bool i2cReadBytes(uint8_t reg, uint8_t *buf, size_t len) {
  Wire.beginTransmission(MAX30102_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)MAX30102_ADDR, (int)len) != (int)len) return false;
  for (size_t i = 0; i < len; i++) buf[i] = Wire.read();
  return true;
}

static bool max30102Init() {
  uint8_t part = 0, rev = 0;
  if (!i2cRead8(REG_PART_ID, part) || !i2cRead8(REG_REV_ID, rev)) {
    Serial.println("MAX30102: I2C read failed.");
    return false;
  }
  Serial.print("MAX30102 PartID=0x"); Serial.print(part, HEX);
  Serial.print(" RevID=0x"); Serial.println(rev, HEX);

  // Reset
  i2cWrite8(REG_MODE_CONFIG, 0x40);
  delay(50);

  // Disable interrupts (polling)
  i2cWrite8(REG_INTR_ENABLE_1, 0x00);
  i2cWrite8(REG_INTR_ENABLE_2, 0x00);

  // FIFO: sample avg=4, rollover=1
  i2cWrite8(REG_FIFO_CONFIG, (0b010 << 5) | (1 << 4) | 0x00);

  // Mode: SpO2 (RED+IR)
  i2cWrite8(REG_MODE_CONFIG, 0x03);

  // SPO2 config: range=4096nA, 100sps, 18bit
  i2cWrite8(REG_SPO2_CONFIG, (0b01 << 5) | (0b001 << 2) | 0b11);

  // LED currents
  i2cWrite8(REG_LED1_PA, 0x24); // RED
  i2cWrite8(REG_LED2_PA, 0x24); // IR

  // SLOT1=RED, SLOT2=IR
  i2cWrite8(REG_MULTI_LED_CTRL1, (0b010 << 4) | 0b001);
  i2cWrite8(REG_MULTI_LED_CTRL2, 0x00);

  // Clear FIFO pointers
  i2cWrite8(REG_FIFO_WR_PTR, 0x00);
  i2cWrite8(REG_OVF_COUNTER, 0x00);
  i2cWrite8(REG_FIFO_RD_PTR, 0x00);

  // Clear pending status
  uint8_t s1, s2;
  i2cRead8(REG_INTR_STATUS_1, s1);
  i2cRead8(REG_INTR_STATUS_2, s2);

  Serial.println("MAX30102 init done.");
  return true;
}

static int max30102AvailableSamples() {
  uint8_t wr = 0, rd = 0;
  if (!i2cRead8(REG_FIFO_WR_PTR, wr)) return 0;
  if (!i2cRead8(REG_FIFO_RD_PTR, rd)) return 0;
  return (int)((wr - rd) & 0x1F); // 5-bit ring
}

static bool max30102ReadSample(uint32_t &red, uint32_t &ir) {
  uint8_t buf[6];
  if (!i2cReadBytes(REG_FIFO_DATA, buf, 6)) return false;

  uint32_t r = ((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8) | buf[2];
  uint32_t i = ((uint32_t)buf[3] << 16) | ((uint32_t)buf[4] << 8) | buf[5];

  red = (r & 0x3FFFF); // 18-bit
  ir  = (i & 0x3FFFF);
  return true;
}

/* ======================== 信号处理 + HR/SpO2 ======================== */

const int SAMPLE_RATE_HZ = 100;

// IIR 状态
static float redDC   = 0.0f;
static float irDC    = 0.0f;
static float redAcF  = 0.0f;
static float irAcF   = 0.0f;
static float redAcSq = 0.0f;
static float irAcSq  = 0.0f;
static float irAbsMean = 0.0f;

// IIR 参数
const float DC_ALPHA      = 0.02f;
const float LP_ALPHA      = 0.20f;
const float RMS_ALPHA     = 0.05f;
const float MEANABS_ALPHA = 0.03f;

// 阈值范围（基础值）
const float THRESH_MIN_BASE = 20.0f;
const float THRESH_MAX      = 800.0f;

// 手指状态 & 时间
static bool   fingerActive        = false;
static unsigned long fingerActiveStartMs = 0;

// 前 5s：1s 预热 + 4s 区间累计
const unsigned long WARMUP_MS           = 1000;  // 第 1 s 只收敛 IIR
const unsigned long INITIAL_TOTAL_MS    = 5000;  // 接触后前 5 s 属于“初始阶段”
const unsigned long INIT_INTERVAL_MS    = 4000;  // 从“第一次峰”起累计的 4s 区间

// 初始区间统计（从第一次峰算起 4s）
static bool   initPhase        = false;
static unsigned long initFirstBeatMs = 0;
static unsigned int  initBeatCount   = 0;

// 预热状态
static bool warmup      = false;
static unsigned long warmupEndMs = 0;

// 手指接触判定
const uint32_t SIG_ON_THRESHOLD  = 100000;
const uint32_t SIG_OFF_THRESHOLD = 80000;

// 滑动窗口用的脉搏时间戳（用于 5s/2s HR）
static unsigned long pulseTimes[32];
static int pulseWriteIndex = 0;
static int pulseStored     = 0;

// 峰检测状态
static float prev2 = 0.0f, prev1 = 0.0f;
static unsigned long lastPulseMs = 0;

// beat-to-beat HR（供 5s 后使用）
static unsigned long lastBeatMs   = 0;
static float hrFast               = 0.0f;
static bool hrFastValid           = false;
static unsigned long hrFastTimeMs = 0;

// 输出值
static float lastHeartRate = 0.0f;
static float lastSpO2      = 0.0f;
static uint32_t lastRawRed = 0;
static uint32_t lastRawIr  = 0;

// --- 显示层小窗口平滑（UI 层，3 点均值） ---
static float hrDisplayBuf[3] = {0};  // 最近 3 次 HR
static int   hrDisplayCount  = 0;    // 有效个数
static int   hrDisplayIndex  = 0;    // 环形写指针

// 滑动窗口
const unsigned long HR_WINDOW_LONG_MS  = 5000;  // 5s
const unsigned long HR_WINDOW_SHORT_MS = 2000;  // 2s

// 不应期
const unsigned long REFRACTORY_MS = 350;

// HR 限速
const float HR_STEP_LIMIT_INIT = 80.0f;  // 前 5 秒
const float HR_STEP_LIMIT_NORM = 40.0f;  // 5 秒之后

static void resetInitPhase() {
  initPhase        = false;
  initFirstBeatMs  = 0;
  initBeatCount    = 0;
}

// 注册脉搏到滑动窗口（预热结束后就可以）
static void registerPulse(unsigned long t_ms) {
  if (!fingerActive || warmup) return;
  pulseTimes[pulseWriteIndex] = t_ms;
  pulseWriteIndex = (pulseWriteIndex + 1) % 32;
  if (pulseStored < 32) pulseStored++;
}

// 计算 HR（与你原来的版本一致）
static void computeHeartRate(unsigned long nowMs) {
  if (!fingerActive) {
    lastHeartRate = 0.0f;
    pulseStored   = 0;
    hrFastValid   = false;
    resetInitPhase();
    return;
  }

  unsigned long contactDuration = nowMs - fingerActiveStartMs;

  // 预热期：不输出 HR
  if (warmup) {
    lastHeartRate = 0.0f;
    return;
  }

  bool inInitialTime = (contactDuration < INITIAL_TOTAL_MS);
  float hrNew = 0.0f;

  // ================== 接触后前 5 秒 ==================
  if (inInitialTime) {
    // 使用“从第一次峰起的 4s 区间累计均值”
    if (initPhase) {
      unsigned long intervalMs = nowMs - initFirstBeatMs;
      if (intervalMs > 0 && intervalMs <= INIT_INTERVAL_MS && initBeatCount >= 2) {
        float durSec = intervalMs / 1000.0f;
        hrNew = (float)initBeatCount * 60.0f / durSec;
      } else if (intervalMs > INIT_INTERVAL_MS) {
        // 超过 4s 就结束初始区间，后面进正常逻辑
        resetInitPhase();
      }
    }

    // 区间均值无效，就先不更新 HR（保持上一次）
    if (hrNew <= 0.0f || hrNew < 30.0f || hrNew > 220.0f) {
      return;
    }

    // 前 5 秒：无平滑，只限速 80
    if (lastHeartRate <= 0.0f) {
      lastHeartRate = hrNew;  // 冷启动
    } else {
      float diff = hrNew - lastHeartRate;
      if (diff > HR_STEP_LIMIT_INIT)  diff = HR_STEP_LIMIT_INIT;
      if (diff < -HR_STEP_LIMIT_INIT) diff = -HR_STEP_LIMIT_INIT;
      lastHeartRate += diff;          // 直接走过去，不做平均
    }
    return;
  }

  // ================== 5 秒之后：滑动窗口 + 权重 ==================
  // 此时初始区间结束，不再用累计平均
  if (initPhase) {
    resetInitPhase();
  }

  // 统计滑动窗口
  int countLong  = 0;
  int countShort = 0;
  for (int i = 0; i < pulseStored; ++i) {
    unsigned long dt = nowMs - pulseTimes[i];
    if (dt <= HR_WINDOW_LONG_MS)  countLong++;
    if (dt <= HR_WINDOW_SHORT_MS) countShort++;
  }

  float hrLong  = 0.0f;
  float hrShort = 0.0f;

  if (countLong >= 2) {
    hrLong = (float)countLong * 60.0f / (HR_WINDOW_LONG_MS / 1000.0f);
  }
  if (countShort >= 2) {
    hrShort = (float)countShort * 60.0f / (HR_WINDOW_SHORT_MS / 1000.0f);
  }

  // beat-to-beat 快 HR，最近 5 秒内有效
  float hrFastUse = 0.0f;
  bool fastFresh  = false;
  if (hrFastValid && (nowMs - hrFastTimeMs) <= 5000) {
    hrFastUse = hrFast;
    fastFresh = true;
  }

  // 老方法：fast + long 加权
  if (hrLong > 0.0f && fastFresh) {
    if (contactDuration < 10000) {
      hrNew = 0.5f * hrFastUse + 0.5f * hrLong;     // 5–10 s：fast 权重大一点
    } else {
      hrNew = 0.3f * hrFastUse + 0.7f * hrLong;     // 10 s 后：偏重 long 稳定
    }
  } else if (hrLong > 0.0f) {
    hrNew = hrLong;
  } else if (fastFresh) {
    hrNew = hrFastUse;
  } else {
    // 没有新 HR：保持旧值，不归零
    return;
  }

  // 合理区间
  if (hrNew <= 0.0f || hrNew < 30.0f || hrNew > 220.0f) {
    return;
  }

  // 5 秒后：限速 40 + 平滑
  if (lastHeartRate <= 0.0f) {
    lastHeartRate = hrNew;
  } else {
    float diff = hrNew - lastHeartRate;
    if (diff > HR_STEP_LIMIT_NORM)  diff = HR_STEP_LIMIT_NORM;
    if (diff < -HR_STEP_LIMIT_NORM) diff = -HR_STEP_LIMIT_NORM;
    float candidate = lastHeartRate + diff;
    lastHeartRate = 0.5f * lastHeartRate + 0.5f * candidate;
  }
}

// 每个样本处理：滤波 + 峰检测 + 状态（与你原来一致）
static void processSample(uint32_t rawRed, uint32_t rawIr, unsigned long t_ms) {
  lastRawRed = rawRed;
  lastRawIr  = rawIr;

  float red = (float)rawRed;
  float ir  = (float)rawIr;

  // 1) DC
  redDC += DC_ALPHA * (red - redDC);
  irDC  += DC_ALPHA * (ir  - irDC);

  float redAc = red - redDC;
  float irAc  = ir  - irDC;

  // 2) AC 低通
  redAcF += LP_ALPHA * (redAc - redAcF);
  irAcF  += LP_ALPHA * (irAc  - irAcF);

  // 3) RMS 平方
  redAcSq += RMS_ALPHA * (redAc * redAc - redAcSq);
  irAcSq  += RMS_ALPHA * (irAc  * irAc  - irAcSq);

  // 4) 动态阈值基准
  irAbsMean += MEANABS_ALPHA * (fabsf(irAcF) - irAbsMean);

  // 5) 手指接触状态
  bool bigSignal   = (rawRed > SIG_ON_THRESHOLD && rawIr > SIG_ON_THRESHOLD);
  bool smallSignal = (rawRed < SIG_OFF_THRESHOLD || rawIr < SIG_OFF_THRESHOLD);

  if (!fingerActive) {
    if (bigSignal) {
      fingerActive         = true;
      fingerActiveStartMs  = t_ms;
      warmup               = true;
      warmupEndMs          = t_ms + WARMUP_MS;
      pulseStored          = 0;
      lastHeartRate        = 0.0f;
      lastSpO2             = 0.0f;
      hrFastValid          = false;
      lastBeatMs           = 0;
      lastPulseMs          = 0;
      resetInitPhase();
    }
  } else {
    if (smallSignal) {
      fingerActive   = false;
      warmup         = false;
      pulseStored    = 0;
      lastHeartRate  = 0.0f;
      lastSpO2       = 0.0f;
      hrFastValid    = false;
      lastBeatMs     = 0;
      lastPulseMs    = 0;
      resetInitPhase();
    } else {
      if (warmup && t_ms >= warmupEndMs) {
        warmup = false;
      }
    }
  }

  // 6) 峰值阈值：1 s 内没峰则降阈 + 刚接触前 8s 下调 THRESH_MIN
  float threshFactor = 0.3f;
  if (fingerActive) {
    if (lastPulseMs == 0 || (t_ms - lastPulseMs > 1000)) {  // 1 s 无峰 → 降阈
      threshFactor = 0.2f;
    }
  }

  float localThreshMin = THRESH_MIN_BASE; // 默认 20
  if (fingerActive) {
    unsigned long contactDuration = t_ms - fingerActiveStartMs;
    if (contactDuration < 8000) {        // 刚接触的前 8 秒
      localThreshMin = 10.0f;            // 降到 10
    }
  }

  float threshold = irAbsMean * threshFactor;
  if (threshold < localThreshMin) threshold = localThreshMin;   // 用动态下限
  if (threshold > THRESH_MAX)     threshold = THRESH_MAX;

  // 7) 三点局部峰
  if (prev1 > prev2 && prev1 > irAcF && prev1 > threshold) {
    unsigned long now = t_ms;
    if (now - lastPulseMs > REFRACTORY_MS) {
      lastPulseMs = now;

      if (fingerActive) {
        unsigned long contactDuration = now - fingerActiveStartMs;

        if (!warmup) {
          // beat-to-beat HR，用于 5 秒后的滑动阶段
          if (lastBeatMs > 0) {
            unsigned long rr = now - lastBeatMs;
            if (rr >= 400 && rr <= 1500) {
              float hr = 60000.0f / (float)rr;
              if (hr >= 30.0f && hr <= 220.0f) {
                hrFast       = hr;
                hrFastValid  = true;
                hrFastTimeMs = now;
              }
            }
          }
          lastBeatMs = now;

          // 初始阶段：从第一次峰起算 4s 区间
          if (contactDuration >= WARMUP_MS && contactDuration < INITIAL_TOTAL_MS) {
            if (!initPhase) {
              initPhase        = true;
              initFirstBeatMs  = now;
              initBeatCount    = 1;
            } else {
              // 还在 4s 区间内就继续累加
              if (now - initFirstBeatMs <= INIT_INTERVAL_MS) {
                initBeatCount++;
              }
            }
          }

          // 同时把脉搏记入滑动窗口（给 5s 之后用）
          registerPulse(now);
        } else {
          // 预热期只更新 lastBeatMs
          lastBeatMs = now;
        }
      }
    }
  }

  prev2 = prev1;
  prev1 = irAcF;
}

// === SpO2：原样 ===
static void updateSpO2() {
  if (!fingerActive) {
    lastSpO2 = 0.0f;
    return;
  }

  float DCred    = redDC;
  float DCir     = irDC;
  float ACredRms = sqrtf(redAcSq);
  float ACirRms  = sqrtf(irAcSq);

  if (DCred > 5000.0f && DCir > 5000.0f &&
      ACredRms > 10.0f && ACirRms > 10.0f) {

    float R = (ACredRms / DCred) / (ACirRms / DCir);
    float spo2 = 104.0f - 17.0f * R;
    if (spo2 > 100.0f) spo2 = 100.0f;
    if (spo2 < 0.0f)   spo2 = 0.0f;
    lastSpO2 = spo2;
  } else {
    lastSpO2 = 0.0f;
  }
}

// UI 层：3 点均值（轻微平滑显示用）
static float getSmoothedHeartRate(float rawHr) {
  if (rawHr <= 0.0f) {
    hrDisplayCount = 0;
    hrDisplayIndex = 0;
    for (int i = 0; i < 3; ++i) hrDisplayBuf[i] = 0.0f;
    return 0.0f;
  }

  hrDisplayBuf[hrDisplayIndex] = rawHr;
  hrDisplayIndex = (hrDisplayIndex + 1) % 3;
  if (hrDisplayCount < 3) hrDisplayCount++;

  float sum = 0.0f;
  for (int i = 0; i < hrDisplayCount; ++i) {
    sum += hrDisplayBuf[i];
  }
  return sum / (float)hrDisplayCount;
}

/* ============================ Arduino ============================ */

void setup() {
  Serial.begin(115200);
  delay(200);

  Serial.println("\nESP32-C3 + MAX30102 + BLE (HR & SpO2)");
  Wire.begin(I2C_SDA, I2C_SCL);  // SDA=GPIO10, SCL=GPIO8
  Wire.setClock(I2C_FREQ);

  if (!max30102Init()) {
    Serial.println("Init failed. Halting.");
    while (true) delay(1000);
  }

  Serial.println("Header: t_ms,raw_red,raw_ir,heartRate_bpm,SpO2_percent");

  // ==== 新增：BLE 初始化 ====
  Serial.println("[BLE] Init BLE server...");
  BLEDevice::init(BLE_SERVER_NAME);           // 设备广播名 "ppg"

  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  BLEService *service = pServer->createService(SERVICE_UUID);

  pCharacteristic = service->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ   |
                      BLECharacteristic::PROPERTY_NOTIFY
                    );

  pCharacteristic->setValue("0,0");          // 初始值：HR=0,SpO2=0
  service->start();

  BLEAdvertising *advertising = BLEDevice::getAdvertising();
  advertising->addServiceUUID(SERVICE_UUID);
  advertising->setScanResponse(true);
  BLEDevice::startAdvertising();

  Serial.println("[BLE] Advertising started, waiting for central...");
}

void loop() {
  unsigned long now = millis();

  int n = max30102AvailableSamples();
  while (n-- > 0) {
    uint32_t red = 0, ir = 0;
    if (max30102ReadSample(red, ir)) {
      processSample(red, ir, now);
    } else {
      Serial.println("Read sample failed");
      break;
    }
  }

  static unsigned long lastPrintMs = 0;
  if (now - lastPrintMs >= 1000) {
    lastPrintMs += 1000;

    computeHeartRate(now);
    updateSpO2();

    float hrToPrint = getSmoothedHeartRate(lastHeartRate);

    // 串口输出
    Serial.print(now);
    Serial.print(",");
    Serial.print(lastRawRed);
    Serial.print(",");
    Serial.print(lastRawIr);
    Serial.print(",");
    Serial.print(hrToPrint, 1);
    Serial.print(",");
    Serial.println(lastSpO2, 1);

    // ==== 新增：BLE 每秒发送一次 “HR,SpO2” ====
    if (deviceConnected) {
      char buf[32];
      // 格式：心率一位小数,血氧一位小数 例： "72.5,97.8"
      snprintf(buf, sizeof(buf), "%.1f,%.1f", hrToPrint, lastSpO2);
      pCharacteristic->setValue((uint8_t*)buf, strlen(buf));
      pCharacteristic->notify();

      Serial.print("[BLE] Notify: ");
      Serial.println(buf);
    }
  }

  delay(5);
}
