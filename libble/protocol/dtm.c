#include <string.h>
#include <ble/protocol/dtm.h>

void ble_dtm_pdu_fill(enum ble_dtm_pattern_e pattern, uint8_t *buffer, size_t size)
{
  if (size < 2)
    return;
  buffer[0] = pattern;
  buffer[1] = size - 2;

  switch (pattern) {
  case BLE_DTM_PATTERN_PRBS9:
    ble_dtm_prbs9_generate(0x1ff, buffer + 2, size - 2);
    break;
  case BLE_DTM_PATTERN_0F:
    memset(buffer + 2, 0x0f, size - 2);
    break;
  case BLE_DTM_PATTERN_55:
    memset(buffer + 2, 0x55, size - 2);
    break;
  case BLE_DTM_PATTERN_PRBS15:
    ble_dtm_prbs15_generate(0x1ff, buffer + 2, size - 2);
    break;
  case BLE_DTM_PATTERN_FF:
    memset(buffer + 2, 0xff, size - 2);
    break;
  case BLE_DTM_PATTERN_00:
    memset(buffer + 2, 0, size - 2);
    break;
  case BLE_DTM_PATTERN_F0:
    memset(buffer + 2, 0xf0, size - 2);
    break;
  case BLE_DTM_PATTERN_AA:
    memset(buffer + 2, 0xaa, size - 2);
    break;
  }
}

uint16_t ble_dtm_prbs9_generate(uint16_t state, uint8_t *buffer, size_t size)
{
  state &= 0x1ff;

  for (size_t i = 0; i < size; ++i) {
    buffer[i] = state;

    for (uint8_t x = 0; x < 8; ++x) {
      state = ((state ^ (state >> 4)) & 1) << 8 | (state >> 1);
    }
  }

  return state;
}

uint16_t ble_dtm_prbs15_generate(uint16_t state, uint8_t *buffer, size_t size)
{
  state &= 0x7fff;

  for (size_t i = 0; i < size; ++i) {
    buffer[i] = state;

    for (uint8_t x = 0; x < 8; ++x) {
      state = ((state ^ (state >> 1)) & 1) << 14 | (state >> 1);
    }
  }

  return state;
}
