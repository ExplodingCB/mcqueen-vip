// CRC-8 SAE J1850: polynomial 0x1D, init 0xFF, no reflection, xorout 0xFF.
// Check value for "123456789" is 0x4B.
#ifndef GATEWAY_CRC8_H
#define GATEWAY_CRC8_H

#include <stddef.h>
#include <stdint.h>

uint8_t gw_crc8(const uint8_t * data, size_t len);

#endif  // GATEWAY_CRC8_H
