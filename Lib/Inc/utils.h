#ifndef __UTILS_H
#define __UTILS_H

// 将0-4095映射到50-250
#include <stdint.h>
uint16_t map_0_4095_to_50_250(uint16_t x);
uint16_t x_map_0_2000_to_50_100(uint16_t x);
uint16_t x_map_2100_4095_to_100_150(uint16_t x);

// y轴
uint16_t y_map_2000_4095_to_0_1000(uint16_t x);
uint16_t y_map_1900_0_to_0_1000(uint16_t x);

#endif