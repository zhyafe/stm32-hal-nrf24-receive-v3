#include <stdint.h>

// 将0-4095映射到50-250
uint16_t map_0_4095_to_50_250(uint16_t x) {
  // 原范围：0-4095
  const uint16_t x_min = 0;
  const uint16_t x_max = 4095;
  // 目标范围：50-250
  const uint16_t y_min = 50;
  const uint16_t y_max = 250;

  // 计算映射值（使用整数运算避免精度损失）
  uint16_t y = y_min + (x - x_min) * (y_max - y_min) / (x_max - x_min);
  return y;
}

// 将0-4095映射到50-250
uint16_t x_map_0_2000_to_50_100(uint16_t x) {
  // 原范围：0-4095
  const uint16_t x_min = 0;
  const uint16_t x_max = 2000;
  // 目标范围：50-250
  const uint16_t y_min = 50;
  const uint16_t y_max = 100;

  // 计算映射值（使用整数运算避免精度损失）
  uint16_t y = y_min + (x - x_min) * (y_max - y_min) / (x_max - x_min);
  return y;
}
uint16_t x_map_2100_4095_to_100_150(uint16_t x) {
  // 原范围：0-4095
  const uint16_t x_min = 2100;
  const uint16_t x_max = 4095;
  // 目标范围：50-250
  const uint16_t y_min = 100;
  const uint16_t y_max = 150;

  // 计算映射值（使用整数运算避免精度损失）
  uint16_t y = y_min + (x - x_min) * (y_max - y_min) / (x_max - x_min);
  return y;
}

uint16_t y_map_2000_4095_to_0_1000(uint16_t x) {
  // 原范围：0-4095
  const uint16_t x_min = 2000;
  const uint16_t x_max = 4095;
  // 目标范围：50-250
  const uint16_t y_min = 0;
  const uint16_t y_max = 1000;

  // 计算映射值（使用整数运算避免精度损失）
  uint16_t y = y_min + (x - x_min) * (y_max - y_min) / (x_max - x_min);
  return y;
}
uint16_t y_map_1900_0_to_0_1000(uint16_t x) {
  uint16_t tmpV = 1900 - x;

  // 原范围：0-4095
  const uint16_t x_min = 0;
  const uint16_t x_max = 1900;
  // 目标范围：50-250
  const uint16_t y_min = 0;
  const uint16_t y_max = 1000;

  // 计算映射值（使用整数运算避免精度损失）
  uint16_t y = y_min + (tmpV - x_min) * (y_max - y_min) / (x_max - x_min);
  return y;
}
