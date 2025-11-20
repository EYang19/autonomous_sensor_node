#include "max30102.h"

static TwoWire* gWire = nullptr;
static uint8_t  gAddr = MAX30102_ADDR;

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

bool readBurst(uint8_t reg, uint8_t* buf, size_t n) {
  gWire->beginTransmission(gAddr);
  gWire->write(reg);
  if (gWire->endTransmission(false) != 0) return false;
  size_t got = gWire->requestFrom((int)gAddr, (int)n);
  for (size_t i=0; i<got; ++i) buf[i] = gWire->read();
  return got == n;
}

void clearFIFO() {
  writeReg(MAX30102::REG_FIFO_WR_PTR, 0x00);
  writeReg(MAX30102::REG_OVF_COUNTER, 0x00);
  writeReg(MAX30102::REG_FIFO_RD_PTR, 0x00);
}

} // anon

namespace MAX30102 {

bool begin(TwoWire& wire, uint8_t addr) {
  gWire = &wire;
  gAddr = addr;
  gWire->begin();
  // 读 PART_ID（0xFF）应为 0x15 :contentReference[oaicite:27]{index=27}
  return (partID() == 0x15);
}

uint8_t partID() {
  uint8_t v=0; readReg(REG_PART_ID, v); return v;
}

void shutdown() {
  uint8_t v; readReg(REG_MODE_CONFIG, v);
  v |= 0x80; // SHDN=1 :contentReference[oaicite:28]{index=28}
  writeReg(REG_MODE_CONFIG, v);
}

void wake() {
  uint8_t v; readReg(REG_MODE_CONFIG, v);
  v &= ~0x80; // SHDN=0
  writeReg(REG_MODE_CONFIG, v);
  // 读一次状态寄存器清 PWR_RDY 中断（上电/掉电后会置位）:contentReference[oaicite:29]{index=29}
  uint8_t dump; readReg(REG_INT_STATUS1, dump);
  readReg(REG_INT_STATUS2, dump);
}

bool configureLowPowerHR(uint8_t sampleRate, uint8_t ledPaIR, uint8_t fifoEmptySlots) {
  wake();
  clearFIFO();

  // FIFO：平均 4，rollover 使能；A_FULL 当剩余空位 = fifoEmptySlots 产生中断（0~15 -> 剩余槽数）:contentReference[oaicite:30]{index=30}
  uint8_t ave = 0x02 << 5; // 010 -> 4 倍平均
  uint8_t rollover = 1 << 4;
  uint8_t a_full = (fifoEmptySlots & 0x0F);
  writeReg(REG_FIFO_CONFIG, ave | rollover | a_full);

  // SPO2 配置：ADC Range=2048nA(00)，采样率根据表选择，LED_PW=215us(10)->17bit（兼顾噪声/功耗）:contentReference[oaicite:31]{index=31}
  uint8_t srBits = 0x01; // 默认 100sps
  switch (sampleRate) {
    case 50:   srBits = 0x00; break;
    case 100:  srBits = 0x01; break;
    case 200:  srBits = 0x02; break;
    case 400:  srBits = 0x03; break;
    case 800:  srBits = 0x04; break;
    case 1000: srBits = 0x05; break;
    default:   srBits = 0x01; break;
  }
  uint8_t spo2 = (0x00 << 5) | (srBits << 2) | (0x02); // ADC_RGE=00, LED_PW=10(215us) :contentReference[oaicite:32]{index=32}
  writeReg(REG_SPO2_CONFIG, spo2);

  // HR 模式：MODE=010，IR only；LED2=IR 电流设置（0~0xFF，约 0~50mA，对应表 8）:contentReference[oaicite:33]{index=33}
  writeReg(REG_MODE_CONFIG, MODE_HR);
  writeReg(REG_LED2_PA, ledPaIR);

  // 使能中断：A_FULL_EN=1
  writeReg(REG_INT_ENABLE1, 0x80); // A_FULL_EN
  writeReg(REG_INT_ENABLE2, 0x00);

  return true;
}

bool configureLowPowerSPO2(uint8_t sampleRate, uint8_t ledPaRed, uint8_t ledPaIR, uint8_t fifoEmptySlots) {
  wake();
  clearFIFO();

  uint8_t ave = 0x02 << 5; // 平均4
  uint8_t rollover = 1 << 4;
  uint8_t a_full = (fifoEmptySlots & 0x0F);
  writeReg(REG_FIFO_CONFIG, ave | rollover | a_full);

  uint8_t srBits = 0x01; // 100sps
  switch (sampleRate) {
    case 50:   srBits = 0x00; break;
    case 100:  srBits = 0x01; break;
    case 200:  srBits = 0x02; break;
    case 400:  srBits = 0x03; break;
    case 800:  srBits = 0x04; break;
    case 1000: srBits = 0x05; break;
    default:   srBits = 0x01; break;
  }
  uint8_t spo2 = (0x00 << 5) | (srBits << 2) | (0x02); // 215us, 17-bit
  writeReg(REG_SPO2_CONFIG, spo2);

  // SpO2 模式：MODE=011，需设置 SLOT1/2 分别为 RED、IR（每样本 6 字节）:contentReference[oaicite:34]{index=34}
  writeReg(REG_MODE_CONFIG, MODE_SPO2);
  writeReg(REG_LED1_PA, ledPaRed);
  writeReg(REG_LED2_PA, ledPaIR);
  // SLOT1=RED(001)，SLOT2=IR(010) :contentReference[oaicite:35]{index=35}
  writeReg(REG_MULTI_LED1, (0x02 << 4) | (0x01));
  writeReg(REG_MULTI_LED2, 0x00);

  writeReg(REG_INT_ENABLE1, 0x80); // A_FULL_EN
  writeReg(REG_INT_ENABLE2, 0x00);
  return true;
}

uint8_t availableSamples() {
  uint8_t wr=0, rd=0;
  readReg(REG_FIFO_WR_PTR, wr);
  readReg(REG_FIFO_RD_PTR, rd);
  int diff = (int)wr - (int)rd;
  if (diff < 0) diff += 32; // FIFO 深度 32 :contentReference[oaicite:36]{index=36}
  return (uint8_t)diff;
}

size_t readFIFO_HR(SampleHR* out, size_t n) {
  size_t i=0;
  while (i<n) {
    uint8_t b[3];
    if (!readBurst(REG_FIFO_DATA, b, 3)) break; // 读此地址不会自增寄存器，但会推进 FIFO 指针 :contentReference[oaicite:37]{index=37}
    out[i].ir24 = ((uint32_t)b[0] << 16) | ((uint32_t)b[1] << 8) | b[2];
    ++i;
  }
  return i;
}

size_t readFIFO_SPO2(SampleSPO2* out, size_t n) {
  size_t i=0;
  while (i<n) {
    uint8_t b[6];
    if (!readBurst(REG_FIFO_DATA, b, 6)) break;
    out[i].red24 = ((uint32_t)b[0] << 16) | ((uint32_t)b[1] << 8) | b[2];
    out[i].ir24  = ((uint32_t)b[3] << 16) | ((uint32_t)b[4] << 8) | b[5];
    ++i;
  }
  return i;
}

} // namespace
