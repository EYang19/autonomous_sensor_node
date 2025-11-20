#pragma once
#include <Arduino.h>
#include <Wire.h>

// MAX30102 7-bit I2C 地址 = 0x57（写 AEh / 读 AFh）:contentReference[oaicite:2]{index=2}
#ifndef MAX30102_ADDR
#define MAX30102_ADDR 0x57
#endif

namespace MAX30102 {

// 重要寄存器/位（摘录）
static constexpr uint8_t REG_INT_STATUS1   = 0x00; // A_FULL, PPG_RDY, ALC_OVF, PWR_RDY  :contentReference[oaicite:3]{index=3}
static constexpr uint8_t REG_INT_STATUS2   = 0x01; // DIE_TEMP_RDY                         :contentReference[oaicite:4]{index=4}
static constexpr uint8_t REG_INT_ENABLE1   = 0x02; // A_FULL_EN, PPG_RDY_EN                :contentReference[oaicite:5]{index=5}
static constexpr uint8_t REG_INT_ENABLE2   = 0x03; // DIE_TEMP_RDY_EN                       :contentReference[oaicite:6]{index=6}
static constexpr uint8_t REG_FIFO_WR_PTR   = 0x04; // FIFO 写指针                          :contentReference[oaicite:7]{index=7}
static constexpr uint8_t REG_OVF_COUNTER   = 0x05; // FIFO 溢出计数                        :contentReference[oaicite:8]{index=8}
static constexpr uint8_t REG_FIFO_RD_PTR   = 0x06; // FIFO 读指针                          :contentReference[oaicite:9]{index=9}
static constexpr uint8_t REG_FIFO_DATA     = 0x07; // FIFO 数据（读此地址重复输出）        :contentReference[oaicite:10]{index=10}
static constexpr uint8_t REG_FIFO_CONFIG   = 0x08; // SMP_AVE, ROLLOVER, A_FULL阈值        :contentReference[oaicite:11]{index=11}
static constexpr uint8_t REG_MODE_CONFIG   = 0x09; // SHDN, RESET, MODE[2:0]               :contentReference[oaicite:12]{index=12}
static constexpr uint8_t REG_SPO2_CONFIG   = 0x0A; // ADC Range, SampleRate, LED_PW        :contentReference[oaicite:13]{index=13}
static constexpr uint8_t REG_LED1_PA       = 0x0C; // 红灯电流（mA 映射表）                :contentReference[oaicite:14]{index=14}
static constexpr uint8_t REG_LED2_PA       = 0x0D; // IR 灯电流                             :contentReference[oaicite:15]{index=15}
static constexpr uint8_t REG_MULTI_LED1    = 0x11; // SLOT1/2                               :contentReference[oaicite:16]{index=16}
static constexpr uint8_t REG_MULTI_LED2    = 0x12; // SLOT3/4                               :contentReference[oaicite:17]{index=17}
static constexpr uint8_t REG_TEMP_INT      = 0x1F; // 片上温度整数                          :contentReference[oaicite:18]{index=18}
static constexpr uint8_t REG_TEMP_FRAC     = 0x20; // 片上温度小数                          :contentReference[oaicite:19]{index=19}
static constexpr uint8_t REG_TEMP_CONFIG   = 0x21; // TEMP_EN 一次转换                      :contentReference[oaicite:20]{index=20}
static constexpr uint8_t REG_PART_ID       = 0xFF; // 应为 0x15                              :contentReference[oaicite:21]{index=21}

enum Mode {
  MODE_HR   = 0x02, // 心率（IR）:contentReference[oaicite:22]{index=22}
  MODE_SPO2 = 0x03  // SpO2（RED+IR）:contentReference[oaicite:23]{index=23}
};

struct SampleHR {      // HR 模式：单通道（IR）18-bit 左对齐，取 3 字节
  uint32_t ir24;       // 仅低 18 位有效（左对齐）
};

struct SampleSPO2 {    // SpO2 模式：双通道（RED+IR），6 字节
  uint32_t red24;
  uint32_t ir24;
};

bool begin(TwoWire& wire = Wire, uint8_t addr = MAX30102_ADDR);
uint8_t partID();

// 低功耗初始化：默认 HR 模式，IR-only，100 sps，LED_PW=215us(17bit)，LED电流低
bool configureLowPowerHR(uint8_t sampleRate = 100, uint8_t ledPaIR = 0x24, uint8_t fifoAlmostFullEmptySlots = 16);

// 可选：配置 SpO2（RED+IR）
bool configureLowPowerSPO2(uint8_t sampleRate = 100, uint8_t ledPaRed = 0x18, uint8_t ledPaIR = 0x24, uint8_t fifoAlmostFullEmptySlots = 8);

// FIFO 可读样本数（基于 WR_PTR/RD_PTR，注意 32 深度环形）:contentReference[oaicite:24]{index=24}
uint8_t availableSamples();

// 读取最多 n 个 HR 样本（每样本 3 字节）
size_t readFIFO_HR(SampleHR* out, size_t n);

// 读取最多 n 个 SpO2 样本（每样本 6 字节：RED、IR）
size_t readFIFO_SPO2(SampleSPO2* out, size_t n);

// 关断（SHDN=1），超低待机电流（典型 0.7 µA）:contentReference[oaicite:25]{index=25}
void shutdown();

// 唤醒（清 SHDN，等待 PWR_RDY 清除）:contentReference[oaicite:26]{index=26}
void wake();

}
