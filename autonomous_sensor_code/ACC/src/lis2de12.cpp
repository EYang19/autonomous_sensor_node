#include "lis2de12.h"

static TwoWire* gWire = nullptr;
static uint8_t  gAddr = LIS2DE12_I2C_ADDR;

namespace {

bool writeReg(uint8_t reg, uint8_t val) {
  gWire->beginTransmission(gAddr);
  gWire->write(reg);
  gWire->write(val);
  return gWire->endTransmission() == 0;
}

bool readReg(uint8_t reg, uint8_t& val) {
  gWire->beginTransmission(gAddr);
  gWire->write(reg);
  if (gWire->endTransmission(false) != 0) return false;
  if (gWire->requestFrom((int)gAddr, 1) != 1) return false;
  val = gWire->read();
  return true;
}

// 连续读 N 字节（自动地址递增，手册规定 SUB(7)=1 可多字节自增）:contentReference[oaicite:13]{index=13}
bool readBurst(uint8_t startReg, uint8_t* buf, size_t n) {
  gWire->beginTransmission(gAddr);
  gWire->write(startReg | 0x80); // MSB=1 -> auto-increment
  if (gWire->endTransmission(false) != 0) return false;
  size_t got = gWire->requestFrom((int)gAddr, (int)n);
  for (size_t i=0; i<got; ++i) buf[i] = gWire->read();
  return got == n;
}

} // anon

namespace LIS2DE12 {

bool begin(TwoWire& wire, uint8_t addr) {
  gWire = &wire;
  gAddr = addr;
  gWire->begin();

  // WHO_AM_I 应为 0x33（手册表 23）:contentReference[oaicite:14]{index=14}
  uint8_t id = whoami();
  return (id == 0x33);
}

uint8_t whoami() {
  uint8_t v=0; readReg(REG_WHO_AM_I, v); return v;
}

float mgPerLSB2g() { return 15.6f; } // ±2g 情况下 15.6 mg/LSB（8-bit）:contentReference[oaicite:15]{index=15}

bool configureLowPowerFIFO(uint8_t watermark) {
  // CTRL1: ODR=10Hz (0010), LPen=1, XYZ=1 -> 0x2F :contentReference[oaicite:16]{index=16}
  if (!writeReg(REG_CTRL1, 0x2F)) return false;

  // CTRL4: BDU=1, FS=±2g -> 0x80（其余置 0）:contentReference[oaicite:17]{index=17}
  if (!writeReg(REG_CTRL4, 0x80)) return false;

  // CTRL5: FIFO_EN=1 -> 0x40 :contentReference[oaicite:18]{index=18}
  if (!writeReg(REG_CTRL5, 0x40)) return false;

  // FIFO_CTRL: Stream 模式 (FM=10)，水位 FTH[4:0]=watermark-1
  uint8_t fth = (uint8_t)max(1, (int)watermark) - 1;
  if (!writeReg(REG_FIFO_CTRL, 0x80 | (fth & 0x1F))) return false; // 0x80 = FM1=1,FM0=0（Stream）:contentReference[oaicite:19]{index=19}

  // CTRL3: 开启 FIFO watermark 中断到 INT1（可选；不接中断也能轮询）:contentReference[oaicite:20]{index=20}
  // I1_WTM 位 = 1 -> 0x04
  writeReg(REG_CTRL3, 0x04);

  return true;
}

uint8_t fifoLevel(uint8_t& flags) {
  uint8_t v=0;
  readReg(REG_FIFO_SRC, v); // EMPTY/WTM/OVRN + FSS[4:0] :contentReference[oaicite:21]{index=21}
  flags = v & 0xE0;
  return (v & 0x1F);
}

size_t readFifoBurst(Sample8* out, size_t maxSamples) {
  // 连续读，从 FIFO_READ_START(0x28) 开始，依次 XH,YH,ZH，自动回卷（手册说明）:contentReference[oaicite:22]{index=22}
  size_t n = maxSamples;
  for (size_t i=0; i<n; ++i) {
    uint8_t buf[6];
    if (!readBurst(REG_FIFO_READ_START, buf, 6)) return i;
    out[i].x = (int8_t)buf[1]; // OUT_X_H 在 +1 偏移（0x29）:contentReference[oaicite:23]{index=23}
    out[i].y = (int8_t)buf[3]; // 0x2B
    out[i].z = (int8_t)buf[5]; // 0x2D
  }
  return n;
}

bool readXYZ(Sample8& s) {
  uint8_t buf[6];
  if (!readBurst(REG_OUT_X_H - 1, buf, 6)) return false; // 从0x28读6字节，H在奇数位
  s.x = (int8_t)buf[1];
  s.y = (int8_t)buf[3];
  s.z = (int8_t)buf[5];
  return true;
}

} // namespace
