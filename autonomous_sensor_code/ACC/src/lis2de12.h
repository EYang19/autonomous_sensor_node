#pragma once
#include <Arduino.h>
#include <Wire.h>

// 默认 I2C 地址：SA0=0 -> 0x18；SA0=1 -> 0x19（手册给出 7bit SAD=001100xb）:contentReference[oaicite:0]{index=0}
#ifndef LIS2DE12_I2C_ADDR
#define LIS2DE12_I2C_ADDR 0x18
#endif

namespace LIS2DE12 {

// 关键寄存器（节选）
static constexpr uint8_t REG_WHO_AM_I     = 0x0F; // 应返回 0x33 :contentReference[oaicite:1]{index=1}
static constexpr uint8_t REG_CTRL0        = 0x1E; // 仅按默认；必要时可断开SA0上拉 :contentReference[oaicite:2]{index=2}
static constexpr uint8_t REG_TEMP_CFG     = 0x1F;
static constexpr uint8_t REG_CTRL1        = 0x20; // ODR[3:0], LPen, Zen, Yen, Xen :contentReference[oaicite:3]{index=3}
static constexpr uint8_t REG_CTRL2        = 0x21; // 高通/滤波选择 :contentReference[oaicite:4]{index=4}
static constexpr uint8_t REG_CTRL3        = 0x22; // INT1 使能（含 FIFO WTM） :contentReference[oaicite:5]{index=5}
static constexpr uint8_t REG_CTRL4        = 0x23; // BDU, FS±2/4/8/16g :contentReference[oaicite:6]{index=6}
static constexpr uint8_t REG_CTRL5        = 0x24; // FIFO_EN 等 :contentReference[oaicite:7]{index=7}
static constexpr uint8_t REG_FIFO_CTRL    = 0x2E; // FM, TR, FTH[4:0] :contentReference[oaicite:8]{index=8}
static constexpr uint8_t REG_FIFO_SRC     = 0x2F; // EMPTY/OVRN/WTM,FSS[4:0] :contentReference[oaicite:9]{index=9}
static constexpr uint8_t REG_FIFO_READ_START = 0x28; // 连读起始地址（6字节循环）:contentReference[oaicite:10]{index=10}

static constexpr uint8_t REG_OUT_X_H      = 0x29; // 8-bit 左对齐高字节（我们用H寄存器读）:contentReference[oaicite:11]{index=11}
static constexpr uint8_t REG_OUT_Y_H      = 0x2B;
static constexpr uint8_t REG_OUT_Z_H      = 0x2D;

struct Sample8 {
  int8_t x, y, z; // 原始8bit，左对齐
};

bool begin(TwoWire& wire = Wire, uint8_t addr = LIS2DE12_I2C_ADDR);

// 读取 FIFO 中可读样本数（0~32）和标志
uint8_t fifoLevel(uint8_t& flags);

// 从 FIFO 连续取出最多 maxSamples 个样本（XYZ 各 1 字节，共 3*maxSamples 字节）
size_t readFifoBurst(Sample8* out, size_t maxSamples);

// 便捷：读取单次（非 FIFO）XYZ
bool readXYZ(Sample8& s);

// 配置为：10 Hz、±2g、BDU=1、FIFO Stream 模式 + 水位 16
bool configureLowPowerFIFO(uint8_t watermark = 16);

// 简单工具
float mgPerLSB2g(); // 约 15.6 mg/LSB（8bit，±2g）:contentReference[oaicite:12]{index=12}
uint8_t whoami();

} // namespace
