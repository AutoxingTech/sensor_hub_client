#pragma once
#include <stdint.h>

/// CRC-16 x16+x15+x2+1  <==> 0x8005
uint16_t calculateCRC16(const uint8_t *p, int len);