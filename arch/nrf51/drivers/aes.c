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
    License along with MutekH; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
    02110-1301 USA.

    Copyright (c), Nicolas Pouillon <nipo@ssji.net>, 2015
*/

#include <mutek/mem_alloc.h>
#include <mutek/kroutine.h>
#include <string.h>

#include <hexo/types.h>
#include <hexo/endian.h>
#include <hexo/lock.h>

#include <device/device.h>
#include <device/resources.h>
#include <device/driver.h>
#include <device/class/crypto.h>
#include <device/irq.h>

#include <arch/nrf51/peripheral.h>
#include <arch/nrf51/ids.h>

#include <arch/nrf51/ecb.h>
#define ECB_ADDR NRF_PERIPHERAL_ADDR(NRF51_ECB)

#ifdef CONFIG_DRIVER_NRF51_AES_CCM
# include <arch/nrf51/ccm.h>
# define CCM_ADDR NRF_PERIPHERAL_ADDR(NRF51_CCM)
#endif

struct nrf51_aes_private_s
{
  struct dev_request_dlqueue_s queue;
#ifdef CONFIG_DRIVER_NRF51_AES_CCM
  struct dev_irq_ep_s irq_ep[1];
  uint8_t scratch[80];
  struct dev_crypto_rq_s *current;
  uint8_t in0;
#endif
};

struct nrf51_aes_state_s
{
  union {
#ifdef CONFIG_DRIVER_NRF51_AES_RANDOM
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

#ifdef CONFIG_DRIVER_NRF51_AES_CMAC
    struct {
      union {
        uint8_t buf8[16];
        uint32_t buf32[4];
      };
      size_t size;
    } cmac;
#endif

#ifdef CONFIG_DRIVER_NRF51_AES_CCM
    struct nrf51_ccm_param_s ccm;
#endif
  };
};

static DEVCRYPTO_INFO(nrf51_aes_info)
{
  switch (accessor->number) {
  default:
    return -ENOENT;

  case 0:
    memset(info, 0, sizeof(*info));

    info->name = "aes";
    info->modes_mask = 0
      | (1 << DEV_CRYPTO_MODE_ECB)
#ifdef CONFIG_DRIVER_NRF51_AES_RANDOM
      | (1 << DEV_CRYPTO_MODE_RANDOM)
#endif
#ifdef CONFIG_DRIVER_NRF51_AES_CMAC
      | (1 << DEV_CRYPTO_MODE_HMAC)
#endif
#ifdef CONFIG_DRIVER_NRF51_AES_CCM
      | (1 << DEV_CRYPTO_MODE_BLE_CCM)
#endif
      ;
    info->cap = DEV_CRYPTO_CAP_INPLACE
      | DEV_CRYPTO_CAP_NOTINPLACE
      | DEV_CRYPTO_CAP_STATEFUL
      | DEV_CRYPTO_CAP_128BITS_KEY
      ;
    info->state_size = sizeof(struct nrf51_aes_state_s);
    info->block_len = 16;
    break;
  }

  return 0;
};

static void nrf51_aes_encrypt_start(struct nrf51_ecb_param_s *params)
{
  nrf_reg_set(ECB_ADDR, NRF51_ECB_ENABLE, NRF51_ECB_ENABLE_ENABLED);
  nrf_reg_set(ECB_ADDR, NRF51_ECB_ECBDATAPTR, (uintptr_t)params);
  nrf_event_clear(ECB_ADDR, NRF51_ECB_ENDECB);
  nrf_task_trigger(ECB_ADDR, NRF51_ECB_STARTECB);
}

static void nrf51_aes_encrypt_spin(void)
{
  nrf_event_wait_clear(ECB_ADDR, NRF51_ECB_ENDECB);
}

static error_t nrf51_aes_encrypt(struct dev_crypto_rq_s *rq)
{
  struct nrf51_ecb_param_s params;

  if (rq->len != 16 || rq->op & DEV_CRYPTO_INVERSE)
    return -ENOTSUP;

  memcpy(params.key, rq->ctx->key_data, 16);
  memcpy(params.cleartext, rq->in, 16);

  nrf51_aes_encrypt_start(&params);
  nrf51_aes_encrypt_spin();

  memcpy(rq->out, params.ciphertext, 16);

  return 0;
}

#ifdef CONFIG_DRIVER_NRF51_AES_RANDOM
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

static void nrf51_aes_drbg_seeded(struct nrf51_aes_state_s *state,
                                  uint8_t *out, size_t out_len,
                                  const uint8_t *ad, size_t ad_len)
{
  struct nrf51_ecb_param_s params1, params2;

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

    nrf51_aes_encrypt_start(&params1);
    for (uint8_t i = 0; i < 4; ++i) {
      params2.key[i] = state->drbg.B[i];
      params2.cleartext[i] = state->drbg.A[i];
    }
    nrf51_aes_encrypt_spin();

    nrf51_aes_encrypt_start(&params2);
    for (uint8_t i = 0; i < 4; ++i)
      state->drbg.A[i] ^= params1.ciphertext[i];
    memcpy(out, params1.ciphertext, out_len_);
    nrf51_aes_encrypt_spin();

    for (uint8_t i = 0; i < 4; ++i)
      state->drbg.B[i] ^= params2.ciphertext[i];

    ad_len -= ad_len_;
    out_len -= out_len_;
    out += out_len_;
  }
}

static error_t nrf51_aes_drbg(struct dev_crypto_rq_s *rq)
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

  nrf51_aes_drbg_seeded(ctx->state_data, out, outlen, ad, adlen);

  return 0;
}
#endif

#ifdef CONFIG_DRIVER_NRF51_AES_CMAC
static error_t nrf51_aes_cmac(struct dev_crypto_rq_s *rq)
{
  struct dev_crypto_context_s *ctx = rq->ctx;
  struct nrf51_aes_state_s *state = ctx->state_data;
  struct nrf51_ecb_param_s params;

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

        nrf51_aes_encrypt_start(&params);
        nrf51_aes_encrypt_spin();

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
    nrf51_aes_encrypt_start(&params);
    nrf51_aes_encrypt_spin();

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

    nrf51_aes_encrypt_start(&params);
    nrf51_aes_encrypt_spin();

    memcpy(rq->out, params.ciphertext, 16);
  }

  return 0;
}
#endif

#ifdef CONFIG_DRIVER_NRF51_AES_CCM
static error_t nrf51_aes_ccm(struct nrf51_aes_private_s *pv,
                             struct dev_crypto_rq_s *rq)
{
  struct dev_crypto_context_s *ctx = rq->ctx;
  struct nrf51_aes_state_s *state = ctx->state_data;

  if (!state)
    return -EINVAL;

  if (rq->op & DEV_CRYPTO_INIT) {
    memcpy(state->ccm.key, ctx->key_data, 16);
    state->ccm.packet_counter = 0;
    state->ccm.direction = (rq->op & DEV_CRYPTO_INVERSE) ? 1 : 0;
    memcpy(state->ccm.iv, rq->iv_ctr, 8);
  }

  if (rq->op & DEV_CRYPTO_FINALIZE) {
    uint8_t *in = (uint8_t*)rq->in - 1;
    uint8_t *out = rq->out - 1;

    pv->current = rq;

    pv->in0 = in[0];
    in[0] = in[1];
    in[1] = in[2];
    in[2] = 0;

    nrf_reg_set(CCM_ADDR, NRF51_CCM_ENABLE, NRF51_CCM_ENABLE_ENABLED);
    nrf_reg_set(CCM_ADDR, NRF51_CCM_MODE, (rq->op & DEV_CRYPTO_INVERSE)
                ? NRF51_CCM_MODE_DECRYPTION
                : NRF51_CCM_MODE_ENCRYPTION);
    nrf_reg_set(CCM_ADDR, NRF51_CCM_CNFPTR, (uintptr_t)state);
    nrf_reg_set(CCM_ADDR, NRF51_CCM_INPTR, (uintptr_t)in);
    nrf_reg_set(CCM_ADDR, NRF51_CCM_OUTPTR, (uintptr_t)out);

    nrf_event_clear(CCM_ADDR, NRF51_CCM_ENDCRYPT);
    nrf_task_trigger(CCM_ADDR, NRF51_CCM_KSGEN);

    return -EAGAIN;
  }

  return 0;
}

static DEV_IRQ_EP_PROCESS(nrf51_aes_irq)
{
  struct device_s *dev = ep->dev;
  struct nrf51_aes_private_s *pv = dev->drv_pv;

  lock_spin(&dev->lock);

  for (;;) {
    struct dev_crypto_rq_s *rq = pv->current;

    if (nrf_event_check(CCM_ADDR, NRF51_CCM_ENDCRYPT)) {
      struct dev_crypto_context_s *ctx = rq->ctx;
      struct nrf51_aes_state_s *state = ctx->state_data;

      nrf_event_clear(CCM_ADDR, NRF51_CCM_ENDCRYPT);
      pv->current = NULL;

      if (rq) {
        uint8_t *in = (uint8_t*)rq->in - 1;
        uint8_t *out = rq->out - 1;

        in[2] = in[1];
        in[1] = in[0];
        in[0] = pv->in0;

        out[2] = out[1];
        out[1] = out[0];

        if (nrf_reg_get(CCM_ADDR, NRF51_CCM_MICSTATUS) == NRF51_CCM_MICSTATUS_FAILED) {
          rq->err = -EINVAL;
        } else {
          rq->err = 0;
          state->ccm.packet_counter++;
        }

        nrf_reg_set(CCM_ADDR, NRF51_CCM_ENABLE, NRF51_CCM_ENABLE_DISABLED);

        dev_request_delayed_end(&pv->queue, &rq->rq);
      }
      continue;
    }

    if (nrf_event_check(CCM_ADDR, NRF51_CCM_ERROR)) {
      nrf_event_clear(CCM_ADDR, NRF51_CCM_ERROR);

      continue;

      pv->current = NULL;

      if (rq) {
        uint8_t *in = (uint8_t*)rq->in - 1;

        in[2] = in[1];
        in[1] = in[0];
        in[0] = pv->in0;

        nrf_reg_set(CCM_ADDR, NRF51_CCM_ENABLE, NRF51_CCM_ENABLE_DISABLED);

        rq->err = -EINVAL;

        dev_request_delayed_end(&pv->queue, &rq->rq);
      }
      continue;
    }

    break;
  }
  
  lock_release(&dev->lock);
}
#endif

static DEV_REQUEST_DELAYED_FUNC(nrf51_aes_process)
{
  struct dev_crypto_rq_s *rq = dev_crypto_rq_s_cast(rq_);
  struct device_s *dev = accessor->dev;
  struct nrf51_aes_private_s *pv = dev->drv_pv;
  struct dev_crypto_context_s *ctx = rq->ctx;

  rq->err = -ENOTSUP;

  switch (accessor->number) {
  default:
    break;

  case 0:
    if (!ctx) {
      rq->err = -EINVAL;
      break;
    }

    switch ((int)ctx->mode) {
    case DEV_CRYPTO_MODE_ECB:
      rq->err = nrf51_aes_encrypt(rq);
      break;

#ifdef CONFIG_DRIVER_NRF51_AES_RANDOM
    case DEV_CRYPTO_MODE_RANDOM:
      rq->err = nrf51_aes_drbg(rq);
      break;
#endif

#ifdef CONFIG_DRIVER_NRF51_AES_CMAC
    case DEV_CRYPTO_MODE_HMAC:
      rq->err = nrf51_aes_cmac(rq);
      break;
#endif

#ifdef CONFIG_DRIVER_NRF51_AES_CCM
    case DEV_CRYPTO_MODE_BLE_CCM:
      rq->err = nrf51_aes_ccm(pv, rq);
      // Special case for IRQ-based processing
      if (rq->err == -EAGAIN)
        return;
      break;
#endif
    }
    break;
  }

  dev_request_delayed_end(&pv->queue, rq_);
}

static DEVCRYPTO_REQUEST(nrf51_aes_request)
{
  struct device_s *dev = accessor->dev;
  struct nrf51_aes_private_s *pv = dev->drv_pv;

  dev_request_delayed_push(device_crypto_s_base(accessor),
                           &pv->queue, dev_crypto_rq_s_base(rq), 1);
}

static DEV_INIT(nrf51_aes_init);
static DEV_CLEANUP(nrf51_aes_cleanup);
#define nrf51_aes_use dev_use_generic

DRIVER_DECLARE(nrf51_aes_drv, "nRF51 AES"
#ifdef CONFIG_DRIVER_NRF51_AES_RANDOM
               ",DRBG"
#endif
#ifdef CONFIG_DRIVER_NRF51_AES_CMAC
               ",CMAC"
#endif
#ifdef CONFIG_DRIVER_NRF51_AES_CCM
               ",CCM"
#endif
               , nrf51_aes,
               DRIVER_CRYPTO_METHODS(nrf51_aes));

DRIVER_REGISTER(nrf51_aes_drv);

static DEV_INIT(nrf51_aes_init)
{
  struct nrf51_aes_private_s  *pv;

  dev->status = DEVICE_DRIVER_INIT_FAILED;

  __unused__ uintptr_t addr = 0;
  assert(device_res_get_uint(dev, DEV_RES_MEM, 0, &addr, NULL) == 0 &&
         ECB_ADDR == addr);

  pv = mem_alloc(sizeof (*pv), (mem_scope_sys));

  if (!pv)
    return -ENOMEM;
  memset(pv, 0, sizeof(*pv));

#ifdef CONFIG_DRIVER_NRF51_AES_CCM
  device_irq_source_init(dev, pv->irq_ep, 1,
                         &nrf51_aes_irq, DEV_IRQ_SENSE_HIGH_LEVEL);

  if (device_irq_source_link(dev, pv->irq_ep, 1, -1))
    goto err_mem;

  nrf_it_disable_mask(CCM_ADDR, -1);

  nrf_it_enable_mask(CCM_ADDR, 0
                     | (1 << NRF51_CCM_ENDCRYPT)
                     | (1 << NRF51_CCM_ERROR));
  nrf_short_enable_mask(CCM_ADDR, 1 << NRF51_CCM_ENDKSGEN_CRYPT);

  nrf_reg_set(CCM_ADDR, NRF51_CCM_SCRATCHPTR, (uintptr_t)pv->scratch);
#endif

  dev->drv_pv = pv;

  dev_request_delayed_init(&pv->queue, &nrf51_aes_process);

  dev->drv = &nrf51_aes_drv;
  dev->status = DEVICE_DRIVER_INIT_DONE;
  return 0;

 err_mem:
  mem_free(pv);
  return -1;
}

static DEV_CLEANUP(nrf51_aes_cleanup)
{
  struct nrf51_aes_private_s  *pv = dev->drv_pv;

  dev_request_delayed_cleanup(&pv->queue);

#ifdef CONFIG_DRIVER_NRF51_AES_CCM
  nrf_it_disable_mask(CCM_ADDR, -1);
  device_irq_source_unlink(dev, pv->irq_ep, 1);
#endif

  mem_free(pv);
}
