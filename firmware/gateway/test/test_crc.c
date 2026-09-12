#include <string.h>

#include "check.h"
#include "gateway/crc8.h"

static int check_value(void)
{
  const char * s = "123456789";
  CHECK(gw_crc8((const uint8_t *)s, strlen(s)) == 0x4Bu);
  return 0;
}

static int single_bit_change_detected(void)
{
  uint8_t a[7] = {1, 2, 3, 4, 5, 6, 7};
  uint8_t crc = gw_crc8(a, 7);
  for (int byte = 0; byte < 7; ++byte) {
    for (int bit = 0; bit < 8; ++bit) {
      uint8_t b[7];
      memcpy(b, a, 7);
      b[byte] ^= (uint8_t)(1u << bit);
      CHECK(gw_crc8(b, 7) != crc);
    }
  }
  return 0;
}

int main(void)
{
  int failures = 0;
  RUN(check_value);
  RUN(single_bit_change_detected);
  return failures ? 1 : 0;
}
