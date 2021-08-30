/*
    This file is part of MutekH.

    MutekH is free software; you can redistribute it and/or modify it
    under the terms of the GNU Lesser General Public License as
    published by the Free Software Foundation; version 2.1 of the
    License.

    MutekH is distributed in the hope that it will be useful, but
    WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this program.  If not, see
    <http://www.gnu.org/licenses/>.

    Copyright Nicolas Pouillon <nipo@ssji.net> (c) 2015
*/

#define LOGK_MODULE_ID "naes"

#include <mutek/mem_alloc.h>
#include <mutek/kroutine.h>
#include <string.h>
#include <mutek/printk.h>

#include <hexo/types.h>
#include <hexo/endian.h>
#include <hexo/lock.h>
#include <hexo/bit.h>

#include <device/device.h>
#include <device/resources.h>
#include <device/driver.h>
#include <device/class/crypto.h>
#include <device/irq.h>

#include <arch/nrf5x/peripheral.h>
#include <arch/nrf5x/ids.h>

#include <arch/nrf5x/ecb.h>
#define ECB_ADDR NRF_PERIPHERAL_ADDR(NRF5X_ECB)

#ifdef CONFIG_DRIVER_NRF5X_AES_CCM
# include <arch/nrf5x/ccm.h>
# define CCM_ADDR NRF_PERIPHERAL_ADDR(NRF5X_CCM)
# include <ble/ccm_params.h>
#endif

DRIVER_PV(struct nrf5x_aes_private_s
{
  struct dev_request_dlqueue_s queue;
#ifdef CONFIG_DRIVER_NRF5X_AES_CCM
  struct dev_irq_src_s irq_ep[1];
  uint8_t scratch[NRF_CCM_SCRATCH_SIZE];
  struct dev_crypto_rq_s *current;
  uint8_t in0;
#endif
});

struct nrf5x_aes_state_s
{
  union {
#ifdef CONFIG_DRIVER_NRF5X_AES_RANDOM
    struct {
      union {
        uint32_t all32[8];
        uint8_t all8[32];
        struct {
          uint32_t A[4];
          uint32_t B[4];
        };
      };
    } drbg;
#endif

#ifdef CONFIG_DRIVER_NRF5X_AES_CMAC
    struct {
      union {
        uint8_t buf8[16];
        uint32_t buf32[4];
      };
      size_t size;
    } cmac;
#endif

#ifdef CONFIG_DRIVER_NRF5X_AES_CCM
    struct nrf5x_ccm_param_s ccm;
#endif
  };
};

static DEV_CRYPTO_INFO(nrf5x_aes_info)
{
  memset(info, 0, sizeof(*info));

  info->name = "aes";
  info->modes_mask = 0
    | bit(DEV_CRYPTO_MODE_ECB)
#ifdef CONFIG_DRIVER_NRF5X_AES_RANDOM
    | bit(DEV_CRYPTO_MODE_RANDOM)
#endif
#ifdef CONFIG_DRIVER_NRF5X_AES_CMAC
    | bit(DEV_CRYPTO_MODE_HMAC)
#endif
#ifdef CONFIG_DRIVER_NRF5X_AES_CCM
    | (1 << DEV_CRYPTO_MODE_BLE_CCM)
#endif
    ;
  info->cap = DEV_CRYPTO_CAP_INPLACE
    | DEV_CRYPTO_CAP_NOTINPLACE
    | DEV_CRYPTO_CAP_STATEFUL
    | DEV_CRYPTO_CAP_128BITS_KEY
    ;
  info->state_size = sizeof(struct nrf5x_aes_state_s);
  info->block_len = 16;

  return 0;
};

static void nrf5x_aes_encrypt_start(struct nrf5x_ecb_param_s *params)
{
  nrf_reg_set(ECB_ADDR, NRF_ECB_ENABLE, NRF_ECB_ENABLE_ENABLED);
  nrf_reg_set(ECB_ADDR, NRF_ECB_ECBDATAPTR, (uintptr_t)params);
  nrf_event_clear(ECB_ADDR, NRF_ECB_ENDECB);
  nrf_task_trigger(ECB_ADDR, NRF_ECB_STARTECB);
}

static void nrf5x_aes_encrypt_spin(void)
{
  nrf_event_wait_clear(ECB_ADDR, NRF_ECB_ENDECB);
}

static error_t nrf5x_aes_encrypt(struct dev_crypto_rq_s *rq)
{
  struct nrf5x_ecb_param_s params;

  if (rq->len != 16 || rq->op & DEV_CRYPTO_INVERSE)
    return -ENOTSUP;

  memcpy(params.key, rq->ctx->key_data, 16);
  memcpy(params.cleartext, rq->in, 16);

  nrf5x_aes_encrypt_start(&params);
  nrf5x_aes_encrypt_spin();

  memcpy(rq->out, params.ciphertext, 16);

  return 0;
}

#ifdef CONFIG_DRIVER_NRF5X_AES_RANDOM
/*
  Global construction:

  /------\     AD     /------\
  |      |     ||     |      |
  |      v     ||     v      |
  |     XOR<---/\--->XOR     |
  |      |            |      |
  |  +----------+---------+  |
  |  [    A     |    B    ]  |
  |  +----------+---------+  |
  |      |            |      |
 XOR<----+----    ----+---->XOR
  ^      |     \/     |      ^
  |      v     /\     v      |
  |     AES<--    -->AES     |
  |     2|            |1     |
  \------/            +------/
                      |
                      v
                     Out
 */

static void nrf5x_aes_drbg_seeded(struct nrf5x_aes_state_s *state,
                                  uint8_t *out, size_t out_len,
                                  const uint8_t *ad, size_t ad_len)
{
  struct nrf5x_ecb_param_s params1, params2;

  while (ad_len || out_len) {
    uint8_t ad_len_ = __MIN(ad_len, 32);
    uint8_t out_len_ = __MIN(out_len, 16);

    if ((ad_len_ & 3) == 0 && ((uintptr_t)ad & 3) == 0) {
      const uint32_t *ad_ = (const uint32_t *)ad;

      for (uint8_t i = 0; i < ad_len_ / 4; ++i)
        state->drbg.all32[i] ^= ad_[i];
    } else {
      for (uint8_t i = 0; i < ad_len_; ++i)
        state->drbg.all8[i] ^= ad[i];
    }

    for (uint8_t i = 0; i < 4; ++i) {
      params1.key[i] = state->drbg.A[i];
      params1.cleartext[i] = state->drbg.B[i];
    }

    nrf5x_aes_encrypt_start(&params1);
    for (uint8_t i = 0; i < 4; ++i) {
      params2.key[i] = state->drbg.B[i];
      params2.cleartext[i] = state->drbg.A[i];
    }
    nrf5x_aes_encrypt_spin();

    nrf5x_aes_encrypt_start(&params2);
    for (uint8_t i = 0; i < 4; ++i)
      state->drbg.A[i] ^= params1.ciphertext[i];
    memcpy(out, params1.ciphertext, out_len_);
    nrf5x_aes_encrypt_spin();

    for (uint8_t i = 0; i < 4; ++i)
      state->drbg.B[i] ^= params2.ciphertext[i];

    ad_len -= ad_len_;
    out_len -= out_len_;
    out += out_len_;
  }
}

static error_t nrf5x_aes_drbg(struct dev_crypto_rq_s *rq)
{
  struct dev_crypto_context_s *ctx = rq->ctx;
  const uint8_t *ad = NULL;
  size_t adlen = 0;
  uint8_t *out = NULL;
  size_t outlen = 0;

  if (rq->op & DEV_CRYPTO_INIT)
    memset(ctx->state_data, 0, 32);

  if (rq->op & DEV_CRYPTO_INVERSE) {
    ad = rq->ad;
    adlen = rq->ad_len;
  }

  if (rq->op & DEV_CRYPTO_FINALIZE) {
    out = rq->out;
    outlen = rq->len;
  }

  nrf5x_aes_drbg_seeded(ctx->state_data, out, outlen, ad, adlen);

  return 0;
}
#endif

#ifdef CONFIG_DRIVER_NRF5X_AES_CMAC
static error_t nrf5x_aes_cmac(struct dev_crypto_rq_s *rq)
{
  struct dev_crypto_context_s *ctx = rq->ctx;
  struct nrf5x_aes_state_s *state = ctx->state_data;
  struct nrf5x_ecb_param_s params;

  memcpy(params.key, rq->ctx->key_data, 16);

  if (rq->op & DEV_CRYPTO_FINALIZE && rq->len != 16)
    return -EINVAL;

  if (rq->op & (DEV_CRYPTO_INIT | DEV_CRYPTO_FINALIZE))
    state = alloca(sizeof(*state));

  if (rq->op & DEV_CRYPTO_INIT)
    memset(state, 0, sizeof(*state));

  if (rq->op & DEV_CRYPTO_INVERSE) {
    const uint8_t *data = rq->ad;
    size_t len = rq->ad_len;

    while (len) {
      size_t align = state->cmac.size & 0xf;
      size_t used = __MIN(16 - align, len);

      if (state->cmac.size && align == 0) {
        for (uint8_t j = 0; j < 4; ++j)
          params.cleartext[j] = state->cmac.buf32[j];

        nrf5x_aes_encrypt_start(&params);
        nrf5x_aes_encrypt_spin();

        for (uint8_t j = 0; j < 4; ++j)
          state->cmac.buf32[j] = params.ciphertext[j];
      }

      memxor(state->cmac.buf8 + align, state->cmac.buf8 + align, data, used);

      data += used;
      state->cmac.size += used;
      len -= used;
    }
  }

  if (rq->op & DEV_CRYPTO_FINALIZE) {
    uint8_t rounds = 1;

    if (state->cmac.size & 0xf || state->cmac.size == 0) {
      rounds = 2;
      state->cmac.buf8[state->cmac.size & 0xf] ^= 0x80;
    }

    memset(params.cleartext, 0, 16);
    nrf5x_aes_encrypt_start(&params);
    nrf5x_aes_encrypt_spin();

    while (rounds--) {
      uint32_t r = *(uint8_t*)params.ciphertext & 0x80 ? 0x87 : 0;

      for (int8_t i = 3; i >= 0; --i) {
        uint32_t s = (int32_t)endian_be32(params.ciphertext[i]) < 8;
        params.ciphertext[i]
          = endian_be32(r ^ (endian_be32(params.ciphertext[i]) << 1));
        r = s;
      }
    }

    for (int8_t i = 0; i < 4; ++i)
      params.cleartext[i] = params.ciphertext[i] ^ state->cmac.buf32[i];

    nrf5x_aes_encrypt_start(&params);
    nrf5x_aes_encrypt_spin();

    memcpy(rq->out, params.ciphertext, 16);
  }

  return 0;
}
#endif

#ifdef CONFIG_DRIVER_NRF5X_AES_CCM
static void nrf5x_aes_ccm_hw_start(struct nrf5x_aes_private_s *pv,
                                   struct dev_crypto_rq_s *rq)
{
  struct dev_crypto_context_s *ctx = rq->ctx;
  struct nrf5x_aes_state_s *state = ctx->state_data;
  const struct ble_ccm_state_s *pstate = (void*)rq->iv_ctr;
  uint8_t *in = (uint8_t*)rq->in - 1;
  uint8_t *out = rq->out - 1;
#if 52000 <= CONFIG_NRF5X_MODEL && CONFIG_NRF5X_MODEL <= 52999
  uint8_t ciphertext_size = rq->in[1] + (rq->op & DEV_CRYPTO_INVERSE ? 0 : 4);
#endif

  pv->current = rq;

  state->ccm.packet_counter = pstate->packet_counter;
  state->ccm.direction = pstate->sent_by_master;

  pv->in0 = in[0];
  in[0] = in[1];
  in[1] = in[2];
  in[2] = 0;

  order_io_mem();

  nrf_reg_set(CCM_ADDR, NRF_CCM_ENABLE, NRF_CCM_ENABLE_ENABLED);
  nrf_reg_set(CCM_ADDR, NRF_CCM_MODE, ((rq->op & DEV_CRYPTO_INVERSE)
                                       ? NRF_CCM_MODE_DECRYPTION_FASTEST
                                       : NRF_CCM_MODE_ENCRYPTION)
#if 52000 <= CONFIG_NRF5X_MODEL && CONFIG_NRF5X_MODEL <= 52999
              | (ciphertext_size > 31 ? NRF_CCM_MODE_LENGTH_8BIT : NRF_CCM_MODE_LENGTH_5BIT)
#endif
              );
  nrf_reg_set(CCM_ADDR, NRF_CCM_CNFPTR, (uintptr_t)state);
  nrf_reg_set(CCM_ADDR, NRF_CCM_INPTR, (uintptr_t)in);
  nrf_reg_set(CCM_ADDR, NRF_CCM_OUTPTR, (uintptr_t)out);
  nrf_reg_set(CCM_ADDR, NRF_CCM_SCRATCHPTR, (uintptr_t)pv->scratch);

  nrf_event_clear(CCM_ADDR, NRF_CCM_ERROR);
  nrf_event_clear(CCM_ADDR, NRF_CCM_ENDCRYPT);
  nrf_task_trigger(CCM_ADDR, NRF_CCM_KSGEN);
}

#if NRF_CCM_MAX_SW_PACKET_SIZE != NRF_CCM_MAX_HW_PACKET_SIZE
static error_t nrf5x_aes_ccm_sw_encrypt(struct nrf5x_aes_private_s *pv,
                                     struct dev_crypto_rq_s *rq)
{
  struct dev_crypto_context_s *ctx = rq->ctx;
  struct nrf5x_aes_state_s *state = ctx->state_data;
  const struct ble_ccm_state_s *pstate = (const void *)rq->iv_ctr;
  struct nrf5x_ecb_param_s aes_ctr, aes_cbc;
  uint32_t len = rq->in[1];

  if (rq->in[1] > 252)
    return -EINVAL;

  for (uint32_t i = 0; i < 4; ++i)
    aes_ctr.key[i] = aes_cbc.key[i] = state->ccm.key[i];

  // Actually, CTR input has same structure than CBC-MAC first block,
  // reuse context.

  // CBC-MAC block 0 (parameters, IV, packet length)
  aes_ctr.cleartext[0] = 0x49 | (pstate->packet_counter << 8);
  aes_ctr.cleartext[1] = (pstate->packet_counter >> 24)
    | (pstate->sent_by_master ? 0x8000 : 0)
    | (state->ccm.iv[0] << 16)
    | (state->ccm.iv[1] << 24);
  aes_ctr.cleartext[2] = endian_le32_na_load(state->ccm.iv + 2);
  aes_ctr.cleartext[3] = endian_le16_na_load(state->ccm.iv + 6)
    | (len << 24);
  nrf5x_aes_encrypt_start(&aes_ctr);
  nrf5x_aes_encrypt_spin();

  // CBC-MAC block 1 (AD)
  aes_cbc.cleartext[0] = aes_ctr.ciphertext[0] ^ (0x100 | ((rq->in[0] & 0xe3) << 16));
  aes_cbc.cleartext[1] = aes_ctr.ciphertext[1];
  aes_cbc.cleartext[2] = aes_ctr.ciphertext[2];
  aes_cbc.cleartext[3] = aes_ctr.ciphertext[3];
  nrf5x_aes_encrypt_start(&aes_cbc);

  *(uint8_t *)aes_ctr.cleartext = 0x01;
  uint16_t *ctr = (uint16_t *)aes_ctr.cleartext + 7;
  *ctr = endian_be16(1);

  nrf5x_aes_encrypt_spin();

  // CTR block 1 (Data cipher stream)
  nrf5x_aes_encrypt_start(&aes_ctr);
  
  for (uint32_t point = 0; point < len; point += 16) {
    uint32_t input[4];

    // CTR stream generation is running, fetch data in the mean time
    memcpy(input, rq->in + 2 + point, __MIN(len - point, 16));
    if (point + 16 > len) {
      size_t offset = len - point;
      memset((uint8_t *)input + offset, 0, 16 - offset);
    }

    // Feedback CBC-MAC input
    aes_cbc.cleartext[0] = aes_cbc.ciphertext[0] ^ input[0];
    aes_cbc.cleartext[1] = aes_cbc.ciphertext[1] ^ input[1];
    aes_cbc.cleartext[2] = aes_cbc.ciphertext[2] ^ input[2];
    aes_cbc.cleartext[3] = aes_cbc.ciphertext[3] ^ input[3];

    // End of CTR
    nrf5x_aes_encrypt_spin();

    // Start CBC-MAC on packet data
    nrf5x_aes_encrypt_start(&aes_cbc);

    // Prepare next counter round
    if (point + 16 < len)
      *ctr = endian_be16(point / 16 + 2);
    else
      *ctr = 0;

    // Apply stream cipher on packet payload
    aes_ctr.ciphertext[0] ^= input[0];
    aes_ctr.ciphertext[1] ^= input[1];
    aes_ctr.ciphertext[2] ^= input[2];
    aes_ctr.ciphertext[3] ^= input[3];

    memcpy(rq->out + 2 + point, aes_ctr.ciphertext, __MIN(len - point, 16));

    // End of CBC-MAC
    nrf5x_aes_encrypt_spin();

    // Next CTR round
    nrf5x_aes_encrypt_start(&aes_ctr);
  }    

  // End of CTR block 0, for MIC encoding
  nrf5x_aes_encrypt_spin();

  rq->out[0] = rq->in[0];
  rq->out[1] = rq->in[1] + 4;
  
  endian_le32_na_store(rq->out + 2 + len, aes_cbc.ciphertext[0] ^ aes_ctr.ciphertext[0]);

  return 0;
}

static error_t nrf5x_aes_ccm_sw_decrypt(struct nrf5x_aes_private_s *pv,
                                        struct dev_crypto_rq_s *rq)
{
  struct dev_crypto_context_s *ctx = rq->ctx;
  struct nrf5x_aes_state_s *state = ctx->state_data;
  const struct ble_ccm_state_s *pstate = (const void *)rq->iv_ctr;
  struct nrf5x_ecb_param_s aes_ctr, aes_cbc;
  uint32_t len = rq->in[1] - 4;

  if (rq->in[1] <= 4)
    return -EINVAL;

  for (uint32_t i = 0; i < 4; ++i)
    aes_ctr.key[i] = aes_cbc.key[i] = state->ccm.key[i];

  // Actually, CTR input has same structure than CBC-MAC first block,
  // reuse context.

  // CBC-MAC block 0 (parameters, IV, packet length)
  aes_ctr.cleartext[0] = 0x49 | (pstate->packet_counter << 8);
  aes_ctr.cleartext[1] = (pstate->packet_counter >> 24)
    | (pstate->sent_by_master ? 0x8000 : 0)
    | (state->ccm.iv[0] << 16)
    | (state->ccm.iv[1] << 24);
  aes_ctr.cleartext[2] = endian_le32_na_load(state->ccm.iv + 2);
  aes_ctr.cleartext[3] = endian_le16_na_load(state->ccm.iv + 6)
    | (len << 24);
  nrf5x_aes_encrypt_start(&aes_ctr);
  nrf5x_aes_encrypt_spin();

  // CBC-MAC block 1 (AD)
  aes_cbc.cleartext[0] = aes_ctr.ciphertext[0] ^ (0x100 | ((rq->in[0] & 0xe3) << 16));
  aes_cbc.cleartext[1] = aes_ctr.ciphertext[1];
  aes_cbc.cleartext[2] = aes_ctr.ciphertext[2];
  aes_cbc.cleartext[3] = aes_ctr.ciphertext[3];
  nrf5x_aes_encrypt_start(&aes_cbc);

  *(uint8_t *)aes_ctr.cleartext = 0x01;
  uint16_t *ctr = (uint16_t *)aes_ctr.cleartext + 7;
  *ctr = endian_be16(1);

  nrf5x_aes_encrypt_spin();

  // CTR block 1 (Data)
  nrf5x_aes_encrypt_start(&aes_ctr);

  for (uint32_t point = 0; point < len; point += 16) {
    uint32_t ciphertext[4];
    uint32_t cleartext[4];

    // CTR stream generation is running, fetch data in the mean time
    memcpy(ciphertext, rq->in + 2 + point, __MIN(len - point, 16));

    // End of CTR
    nrf5x_aes_encrypt_spin();

    // Apply stream cipher on packet payload
    cleartext[0] = aes_ctr.ciphertext[0] ^ ciphertext[0];
    cleartext[1] = aes_ctr.ciphertext[1] ^ ciphertext[1];
    cleartext[2] = aes_ctr.ciphertext[2] ^ ciphertext[2];
    cleartext[3] = aes_ctr.ciphertext[3] ^ ciphertext[3];

    if (point + 16 > len) {
      size_t offset = len - point;
      memset((uint8_t *)cleartext + offset, 0, 16 - offset);
    }

    // Feedback CBC-MAC input
    aes_cbc.cleartext[0] = aes_cbc.ciphertext[0] ^ cleartext[0];
    aes_cbc.cleartext[1] = aes_cbc.ciphertext[1] ^ cleartext[1];
    aes_cbc.cleartext[2] = aes_cbc.ciphertext[2] ^ cleartext[2];
    aes_cbc.cleartext[3] = aes_cbc.ciphertext[3] ^ cleartext[3];

    // Start CBC-MAC on packet data
    nrf5x_aes_encrypt_start(&aes_cbc);

    // Prepare next counter round
    if (point + 16 < len)
      *ctr = endian_be16(point / 16 + 2);
    else
      *ctr = 0;

    memcpy(rq->out + 2 + point, cleartext, __MIN(len - point, 16));

    // End of CBC-MAC
    nrf5x_aes_encrypt_spin();

    // Next CTR round
    nrf5x_aes_encrypt_start(&aes_ctr);
  }    

  // End of CTR block 0, for MIC encoding
  nrf5x_aes_encrypt_spin();

  rq->out[0] = rq->in[0];
  rq->out[1] = rq->in[1] - 4;

  uint32_t mic = aes_cbc.ciphertext[0] ^ aes_ctr.ciphertext[0];

  if (endian_le32_na_load(rq->in + 2 + len) == mic)
    return 0;

  logk_trace("MIC Status failed, expected %08x, got %08x:",
         endian_le32_na_load(rq->in + 2 + len), mic);
  logk_trace(" In:  %P", rq->in, rq->in[1] + 6);
  logk_trace(" Out: %P", rq->out, rq->out[1] + 2);

  return -EINVAL;
}
#endif

static bool_t nrf5x_aes_ccm(struct nrf5x_aes_private_s *pv,
                            struct dev_crypto_rq_s *rq)
{
  struct dev_crypto_context_s *ctx = rq->ctx;
  struct nrf5x_aes_state_s *state = ctx->state_data;

  if (!state) {
    rq->error = -EINVAL;
    return 0;
  }

  rq->error = -ENOTSUP;

  if (rq->op & DEV_CRYPTO_INIT) {
    memcpy(state->ccm.key, ctx->key_data, 16);
    memcpy(state->ccm.iv, rq->iv_ctr, 8);
    rq->error = 0;
  }

  if (rq->op & DEV_CRYPTO_FINALIZE) {
    uint8_t cleartext_size = rq->in[1] - (rq->op & DEV_CRYPTO_INVERSE ? 4 : 0);

    if (cleartext_size <= NRF_CCM_MAX_HW_PACKET_SIZE) {
      rq->error = -EAGAIN;
      nrf5x_aes_ccm_hw_start(pv, rq);
      return 1;
    }

#if NRF_CCM_MAX_SW_PACKET_SIZE != NRF_CCM_MAX_HW_PACKET_SIZE
    if (rq->op & DEV_CRYPTO_INVERSE)
      rq->error = nrf5x_aes_ccm_sw_decrypt(pv, rq);
    else
      rq->error = nrf5x_aes_ccm_sw_encrypt(pv, rq);
#else
    rq->error = -ENOTSUP;
#endif
  }

  return 0;
}

static DEV_IRQ_SRC_PROCESS(nrf5x_aes_irq)
{
  struct device_s *dev = ep->base.dev;
  struct nrf5x_aes_private_s *pv = dev->drv_pv;

  lock_spin(&dev->lock);

  for (;;) {
    if (nrf_event_check(CCM_ADDR, NRF_CCM_ERROR)) {
      nrf_event_clear(CCM_ADDR, NRF_CCM_ERROR);

      logk_trace("CCM bus conflict, restarting");
      nrf_event_clear(CCM_ADDR, NRF_CCM_ENDCRYPT);
      nrf_task_trigger(CCM_ADDR, NRF_CCM_KSGEN);

      continue;
    }

    if (nrf_event_check(CCM_ADDR, NRF_CCM_ENDCRYPT)) {
      nrf_event_clear(CCM_ADDR, NRF_CCM_ENDCRYPT);

      if (!pv->current) {
        logk_trace("Spurious CCM IRQ");
        continue;
      }

      struct dev_crypto_rq_s *rq = pv->current;
      struct dev_crypto_context_s *ctx = rq->ctx;
      __unused__ struct nrf5x_aes_state_s *state = ctx->state_data;

      if ((rq->op & DEV_CRYPTO_INVERSE)
          && nrf_reg_get(CCM_ADDR, NRF_CCM_MICSTATUS) == NRF_CCM_MICSTATUS_FAILED
          && ((nrf_reg_get(CCM_ADDR, NRF_CCM_MODE) & NRF_CCM_MODE_MASK)
              == NRF_CCM_MODE_DECRYPTION_FASTEST)) {

        logk_trace("Fast CCM decrypt failed, starting again in slow mode");
        nrf_reg_set(CCM_ADDR, NRF_CCM_MODE,
                    (nrf_reg_get(CCM_ADDR, NRF_CCM_MODE) & ~NRF_CCM_MODE_MASK)
                    | NRF_CCM_MODE_DECRYPTION);

        nrf_event_clear(CCM_ADDR, NRF_CCM_ENDCRYPT);
        nrf_event_clear(CCM_ADDR, NRF_CCM_ERROR);
        nrf_task_trigger(CCM_ADDR, NRF_CCM_KSGEN);

        continue;
      }

      pv->current = NULL;

      uint8_t *in = (uint8_t*)rq->in - 1;
      uint8_t *out = rq->out - 1;

      in[2] = in[1];
      in[1] = in[0];
      in[0] = pv->in0;

      out[2] = out[1];
      out[1] = out[0];

      if (nrf_reg_get(CCM_ADDR, NRF_CCM_MICSTATUS) == NRF_CCM_MICSTATUS_FAILED) {
        if (rq->op & DEV_CRYPTO_INVERSE) {
          logk_error("MIC Status failed:");
          logk_error(" In:  %P", rq->in, rq->in[1] + 2);
          logk_error(" Out: %P", rq->out, rq->out[1] + 2);
          logk_error(" Ctx: %P", state, sizeof(*state));
          logk_error(" Scr: %P", pv->scratch, sizeof(pv->scratch));
          rq->error = -EINVAL;
        } else {
          rq->error = 0;
        }
      } else {
        rq->error = 0;
      }

      /* order_io_mem(); */

      nrf_reg_set(CCM_ADDR, NRF_CCM_ENABLE, NRF_CCM_ENABLE_DISABLED);

      lock_release(&dev->lock);
      dev_request_delayed_end(&pv->queue, &rq->base);
      lock_spin(&dev->lock);
      continue;
    }

    break;
  }
  
  lock_release(&dev->lock);
}
#endif

static DEV_REQUEST_DELAYED_FUNC(nrf5x_aes_process)
{
  struct dev_crypto_rq_s *rq = dev_crypto_rq_s_cast(rq_);
  struct device_s *dev = accessor->dev;
  struct nrf5x_aes_private_s *pv = dev->drv_pv;
  struct dev_crypto_context_s *ctx = rq->ctx;
  bool_t delayed = 0;

  rq->error = -ENOTSUP;

  if (!ctx) {
    rq->error = -EINVAL;
    goto end;
  }

  switch ((int)ctx->mode) {
  case DEV_CRYPTO_MODE_ECB:
    rq->error = nrf5x_aes_encrypt(rq);
    break;

#ifdef CONFIG_DRIVER_NRF5X_AES_RANDOM
  case DEV_CRYPTO_MODE_RANDOM:
    rq->error = nrf5x_aes_drbg(rq);
    break;
#endif

#ifdef CONFIG_DRIVER_NRF5X_AES_CMAC
  case DEV_CRYPTO_MODE_HMAC:
    rq->error = nrf5x_aes_cmac(rq);
    break;
#endif

#ifdef CONFIG_DRIVER_NRF5X_AES_CCM
  case DEV_CRYPTO_MODE_BLE_CCM:
    delayed = nrf5x_aes_ccm(pv, rq);
    break;
#endif
  }

 end:
  if (!delayed)
    dev_request_delayed_end(&pv->queue, rq_);
}

static DEV_CRYPTO_REQUEST(nrf5x_aes_request)
{
  struct device_s *dev = accessor->dev;
  struct nrf5x_aes_private_s *pv = dev->drv_pv;

  dev_request_delayed_push(device_crypto_s_base((struct device_crypto_s*)accessor),
                           &pv->queue, dev_crypto_rq_s_base(rq), 1);
}

#define nrf5x_aes_use dev_use_generic

static DEV_INIT(nrf5x_aes_init)
{
  struct nrf5x_aes_private_s  *pv;

  __unused__ uintptr_t addr = 0;
  assert(device_res_get_uint(dev, DEV_RES_MEM, 0, &addr, NULL) == 0 &&
         ECB_ADDR == addr);

  pv = mem_alloc(sizeof (*pv), (mem_scope_sys));
  if (!pv)
    return -ENOMEM;

  memset(pv, 0, sizeof(*pv));

#ifdef CONFIG_DRIVER_NRF5X_AES_CCM
  device_irq_source_init(dev, pv->irq_ep, 1, &nrf5x_aes_irq);

  if (device_irq_source_link(dev, pv->irq_ep, 1, -1))
    goto err_mem;

  nrf_it_disable_mask(CCM_ADDR, -1);

  nrf_it_enable_mask(CCM_ADDR, 0
                     | (1 << NRF_CCM_ENDCRYPT)
                     | (1 << NRF_CCM_ERROR));
  nrf_short_enable_mask(CCM_ADDR, 1 << NRF_CCM_ENDKSGEN_CRYPT);
#endif

  dev->drv_pv = pv;

  dev_request_delayed_init(&pv->queue, &nrf5x_aes_process);

  return 0;

 err_mem:
  mem_free(pv);
  return -1;
}

static DEV_CLEANUP(nrf5x_aes_cleanup)
{
  struct nrf5x_aes_private_s  *pv = dev->drv_pv;

  if (!dev_request_delayed_isidle(&pv->queue))
    return -EBUSY;

  dev_request_delayed_cleanup(&pv->queue);

#ifdef CONFIG_DRIVER_NRF5X_AES_CCM
  nrf_it_disable_mask(CCM_ADDR, -1);
  device_irq_source_unlink(dev, pv->irq_ep, 1);
#endif

  mem_free(pv);

  return 0;
}

DRIVER_DECLARE(nrf5x_aes_drv, 0, "nRF5x AES"
#ifdef CONFIG_DRIVER_NRF5X_AES_RANDOM
               ",DRBG"
#endif
#ifdef CONFIG_DRIVER_NRF5X_AES_CMAC
               ",CMAC"
#endif
#ifdef CONFIG_DRIVER_NRF5X_AES_CCM
               ",CCM"
#endif
               , nrf5x_aes,
               DRIVER_CRYPTO_METHODS(nrf5x_aes));

DRIVER_REGISTER(nrf5x_aes_drv);
