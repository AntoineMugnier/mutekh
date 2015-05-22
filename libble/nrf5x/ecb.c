#include <arch/nrf5x/ecb.h>
#include <arch/nrf5x/ccm.h>

#include <string.h>

#include <ble/crypto.h>
#include <ble/packet.h>

/*
  43us / block (for function call)

  Product specification announces 8.5us typical, 17us max
*/
error_t ble_e(const uint8_t *key, const uint8_t *in, uint8_t *out)
{
    uintptr_t ecb = nrf_peripheral_addr(NRF5X_ECB);
    uint8_t param[48];

    memcpy(param, key, 16);
    memcpy(param + 16, in, 16);

    nrf_event_clear(ecb, NRF_ECB_ENDECB);
    nrf_event_clear(ecb, NRF_ECB_ERRORECB);
    nrf_reg_set(ecb, NRF_ECB_ECBDATAPTR, (uint32_t)param);
#if 1
    nrf_task_trigger(ecb, NRF_ECB_STARTECB);

    while (!nrf_event_check(ecb, NRF_ECB_ENDECB)) {
        if (nrf_event_check(ecb, NRF_ECB_ERRORECB)) {
            nrf_event_clear(ecb, NRF_ECB_ERRORECB);
            return -EAGAIN;
        }
    }
#endif

    nrf_event_clear(ecb, NRF_ECB_ENDECB);
    memcpy(out, param + 32, 16);

    return 0;
}

error_t ble_ccm_encrypt(
    const uint8_t *k, const uint8_t *iv, uint32_t packet_count, uint8_t from_master,
    const struct buffer_s *in_, struct buffer_s *out)
{
  uintptr_t ccm = nrf_peripheral_addr(NRF5X_CCM);
  uint8_t ctx[33];
  uint8_t scratch[40 + 16];

  memcpy(ctx, k, 16);
  endian_le32_na_store(ctx + 16, packet_count);
  ctx[20] = ctx[21] = ctx[22] = ctx[23] = 0;
  ctx[24] = from_master;
  memcpy(ctx + 25, iv, 8);

  struct buffer_s *in = (struct buffer_s *)in_;

  assert(in->begin > 0);
  out->begin = 1;

  in->data[in->begin - 1] = in->data[in->begin];
  in->data[in->begin] = in->data[in->begin + 1];
  in->data[in->begin + 1] = 0;

  nrf_reg_set(ccm, NRF_CCM_ENABLE, NRF_CCM_ENABLE_ENABLED);

  nrf_reg_set(ccm, NRF_CCM_CNFPTR, (uint32_t)ctx);
  nrf_reg_set(ccm, NRF_CCM_INPTR, (uint32_t)in->data + in->begin - 1);
  nrf_reg_set(ccm, NRF_CCM_OUTPTR, (uint32_t)out->data + out->begin - 1);
  nrf_reg_set(ccm, NRF_CCM_MODE, NRF_CCM_MODE_ENCRYPTION);
  nrf_reg_set(ccm, NRF_CCM_SCRATCHPTR, (uint32_t)scratch);

  nrf_event_clear(ccm, NRF_CCM_ENDKSGEN);
  nrf_event_clear(ccm, NRF_CCM_ENDCRYPT);
  nrf_short_set(ccm, 1 << NRF_CCM_ENDKSGEN_CRYPT);
  nrf_task_trigger(ccm, NRF_CCM_KSGEN);

  while (!nrf_event_check(ccm, NRF_CCM_ENDCRYPT)) {
    if (nrf_event_check(ccm, NRF_CCM_ERROR)) {
      nrf_event_clear(ccm, NRF_CCM_ERROR);
      return -EIO;
    }
  }

  nrf_event_clear(ccm, NRF_CCM_ENDCRYPT);
  nrf_reg_set(ccm, NRF_CCM_ENABLE, 0);

  in->data[in->begin + 1] = in->data[in->begin];
  in->data[in->begin] = in->data[in->begin - 1];

  out->data[out->begin + 1] = out->data[out->begin];
  out->data[out->begin] = out->data[out->begin - 1];

  out->end = out->data[out->begin + 1] + out->begin;

  return 0;
}

/**
   1us/bit (probably enforced by radio synchronization problematic)
 */
error_t ble_ccm_decrypt(
    const uint8_t *k, const uint8_t *iv, uint32_t packet_count, uint8_t from_master,
    const struct buffer_s *in_, struct buffer_s *out)
{
  uintptr_t ccm = nrf_peripheral_addr(NRF5X_CCM);
  uint8_t ctx[33];
  uint8_t scratch[40 + 16];

  memcpy(ctx, k, 16);
  endian_le32_na_store(ctx + 16, packet_count);
  ctx[20] = ctx[21] = ctx[22] = ctx[23] = 0;
  ctx[24] = !!from_master;
  memcpy(ctx + 25, iv, 8);

  struct buffer_s *in = (struct buffer_s *)in_;

  assert(in->begin > 0);
  out->begin = 1;

  in->data[in->begin - 1] = in->data[in->begin];
  in->data[in->begin] = in->data[in->begin + 1];
  in->data[in->begin + 1] = 0;

  nrf_reg_set(ccm, NRF_CCM_ENABLE, NRF_CCM_ENABLE_ENABLED);

  nrf_reg_set(ccm, NRF_CCM_CNFPTR, (uint32_t)ctx);
  nrf_reg_set(ccm, NRF_CCM_INPTR, (uint32_t)in->data + in->begin - 1);
  nrf_reg_set(ccm, NRF_CCM_OUTPTR, (uint32_t)out->data + out->begin - 1);
  nrf_reg_set(ccm, NRF_CCM_MODE, NRF_CCM_MODE_DECRYPTION);
  nrf_reg_set(ccm, NRF_CCM_SCRATCHPTR, (uint32_t)scratch);

  nrf_event_clear(ccm, NRF_CCM_ENDKSGEN);
  nrf_event_clear(ccm, NRF_CCM_ENDCRYPT);
  nrf_short_set(ccm, 1 << NRF_CCM_ENDKSGEN_CRYPT);
  nrf_task_trigger(ccm, NRF_CCM_KSGEN);

  while (!nrf_event_check(ccm, NRF_CCM_ENDCRYPT)) {
    if (nrf_event_check(ccm, NRF_CCM_ERROR)) {
      nrf_event_clear(ccm, NRF_CCM_ERROR);
      return -EIO;
    }
  }

  nrf_event_clear(ccm, NRF_CCM_ENDCRYPT);

  uint32_t mic_ok = nrf_reg_get(ccm, NRF_CCM_MICSTATUS);

  nrf_reg_set(ccm, NRF_CCM_ENABLE, 0);

  in->data[in->begin + 1] = in->data[in->begin];
  in->data[in->begin] = in->data[in->begin - 1];

  out->data[out->begin] = in->data[in->begin];
  out->data[out->begin + 1] = in->data[in->begin + 1] - 4;
  out->end = out->begin + out->data[out->begin + 1] + 2;

  out->timestamp = in->timestamp;
  out->crc_valid = in->crc_valid;
  out->duration = in->duration;
  out->rssi = in->rssi;
  out->index = in->index;

  return mic_ok ? 0 : -EINVAL;
}
