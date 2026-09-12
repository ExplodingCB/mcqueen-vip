#include "gateway/crc8.h"

uint8_t gw_crc8(const uint8_t * data, size_t len)
{
  uint8_t crc = 0xFFu;
  for (size_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (int bit = 0; bit < 8; ++bit) {
      crc = (uint8_t)((crc & 0x80u) ? (((unsigned)crc << 1) ^ 0x1Du) : ((unsigned)crc << 1));
    }
  }
  return (uint8_t)(crc ^ 0xFFu);
}
