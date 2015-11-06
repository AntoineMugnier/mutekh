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

    Copyright Alexandre Becoulet <alexandre.becoulet@lip6.fr> (c) 2014

*/

#include "aes.h"

#include <mutek/mem_alloc.h>

static DEVCRYPTO_INFO(efm32_aes_info)
{
#if 0
  struct device_s *dev = accessor->dev;
  struct efm32_aes_private_s *pv = dev->drv_pv;
#endif

  memset(info, 0, sizeof(*info));
  info->name = "aes";
  info->modes_mask = 0
#ifdef CONFIG_DRIVER_EFM32_AES_ECB
    | (1 << DEV_CRYPTO_MODE_ECB)
#endif
#ifdef CONFIG_DRIVER_EFM32_AES_CBC
    | (1 << DEV_CRYPTO_MODE_CBC)
#endif
#ifdef CONFIG_DRIVER_EFM32_AES_CTR
    | (1 << DEV_CRYPTO_MODE_CTR)
#endif
#ifdef CONFIG_DRIVER_EFM32_AES_OCB3
    | (1 << DEV_CRYPTO_MODE_OCB3)
#endif
#ifdef CONFIG_DRIVER_EFM32_AES_RANDOM
    | (1 << DEV_CRYPTO_MODE_RANDOM)
#endif
    ;
  info->align_log2 = 0;
  info->cap = 0
    | DEV_CRYPTO_CAP_128BITS_KEY
#ifdef CONFIG_DRIVER_EFM32_AES_KEY256
    | DEV_CRYPTO_CAP_256BITS_KEY
#endif
#ifdef CONFIG_DRIVER_EFM32_AES_AD
    | DEV_CRYPTO_CAP_ASSOC_DATA
#endif
    | DEV_CRYPTO_CAP_INPLACE | DEV_CRYPTO_CAP_NOTINPLACE;

  info->block_len = 16;
  info->state_size = sizeof(struct efm32_aes_state_s);

  return 0;
}

extern inline void efm32_aes_load_key128l(const uint8_t *key);
extern inline void efm32_aes_get_key128(uint8_t *key);
#ifdef EFM32_AES_CTRL_KEYBUFEN
extern inline void efm32_aes_load_key128h(const uint8_t *key);
#endif
#ifdef CONFIG_DRIVER_EFM32_AES_KEY256
extern inline void efm32_aes_load_key256(const uint8_t *key);
extern inline void efm32_aes_get_key256(uint8_t *key);
#endif
extern inline void efm32_aes_wait();

static DEV_REQUEST_DELAYED_FUNC(efm32_aes_process)
{
  struct device_s *dev = accessor->dev;
  struct efm32_aes_private_s *pv = dev->drv_pv;
  struct dev_crypto_rq_s *rq = dev_crypto_rq_s_cast(rq_);
  struct dev_crypto_context_s *ctx = rq->ctx;
  const uint8_t *rawkey;

#ifdef CONFIG_DRIVER_EFM32_AES_RANDOM
  if (ctx->mode == DEV_CRYPTO_MODE_RANDOM)
    {
      struct efm32_aes_state_s * __restrict__ st = ctx->state_data;

      if (rq->op & DEV_CRYPTO_INIT)
        memset(st->rand8, 0, 32);

      size_t l = rq->ad_len;
      if ((rq->op & DEV_CRYPTO_INVERSE) && l)
        {
          const uint8_t * __restrict__ in = rq->ad;
          while (l--)
            {
              uint_fast8_t i = l % 32;
              st->rand8[i] ^= *in++;
              if (!i)
                efm32_aes_random(st->rand32, NULL);
            }
        }

      l = rq->len;
      if ((rq->op & DEV_CRYPTO_FINALIZE) && l)
        {
          uint8_t * __restrict__ out = rq->out;
          union {
            uint32_t r32[4];
            uint8_t r8[16];
          }          rout;
          while (l)
            {
              efm32_aes_random(st->rand32, rout.r32);
              size_t r = __MIN(l, 16);
              l -= r;
              while (r--)
                *out++ = rout.r8[r];
            }
        }
      rq->err = 0;
      goto pop;
    }
#endif

  bool_t ctx_ok = dev_crypto_ctx_bind(ctx, pv->ctx, &pv->next,
                            CONFIG_DRIVER_EFM32_AES_CTXCOUNT);
  struct efm32_aes_context_s *actx = pv->actx + ctx->cache_id;

  rq->err = -ENOTSUP;

  if ((rq->op & DEV_CRYPTO_INVERSE)
#ifdef CONFIG_DRIVER_EFM32_AES_CTR
      && ctx->mode != DEV_CRYPTO_MODE_CTR
#endif
      )
    rawkey = actx->dkey;
  else
    rawkey = ctx->key_data;

#ifndef CONFIG_DRIVER_EFM32_AES_AD
  if (rq->ad_len)
    goto pop;
#endif

  switch (ctx->key_len)
    {
    case 16: {
      if (!ctx_ok && !ctx->encrypt_only)
        {
          cpu_mem_write_32(CONFIG_EFM32_AES_ADDR + EFM32_AES_CTRL_ADDR, 0);
          efm32_aes_load_key128l(ctx->key_data);
          cpu_mem_write_32(CONFIG_EFM32_AES_ADDR + EFM32_AES_CMD_ADDR, EFM32_AES_CMD_START);
          efm32_aes_wait();
          efm32_aes_get_key128(actx->dkey);
        }

#ifdef EFM32_AES_CTRL_KEYBUFEN
      efm32_aes_load_key128h(rawkey);
      rawkey = NULL;
#endif
      break;
    }

#ifdef CONFIG_DRIVER_EFM32_AES_KEY256
    case 32:
      if (!ctx_ok && !ctx->encrypt_only)
        {
          cpu_mem_write_32(CONFIG_EFM32_AES_ADDR + EFM32_AES_CTRL_ADDR, EFM32_AES_CTRL_AES256);
          efm32_aes_load_key256(ctx->key_data);
          cpu_mem_write_32(CONFIG_EFM32_AES_ADDR + EFM32_AES_CMD_ADDR, EFM32_AES_CMD_START);
          efm32_aes_wait();
          efm32_aes_get_key256(actx->dkey);
        }
      break;
#endif
    default:
      goto pop;
    }

  switch (ctx->mode)
    {
#ifdef CONFIG_DRIVER_EFM32_AES_ECB
    case DEV_CRYPTO_MODE_ECB:
      efm32_aes_ecb(actx, rq, rawkey);
      break;
#endif
#ifdef CONFIG_DRIVER_EFM32_AES_CBC
    case DEV_CRYPTO_MODE_CBC:
      if (rq->len & 15)
        goto pop;
      if (ctx->iv_len != 16)
        goto pop;
      if (rq->op & DEV_CRYPTO_INVERSE)
        efm32_aes_cbc_decrypt(actx, rq, rawkey);
      else
        efm32_aes_cbc_encrypt(actx, rq, rawkey);
      break;
#endif
#ifdef CONFIG_DRIVER_EFM32_AES_CTR
    case DEV_CRYPTO_MODE_CTR:
      efm32_aes_ctr(actx, rq, rawkey);
      break;
#endif
#ifdef CONFIG_DRIVER_EFM32_AES_OCB3
    case DEV_CRYPTO_MODE_OCB3: {
      if ((rq->op & (DEV_CRYPTO_INIT | DEV_CRYPTO_FINALIZE))
          != (DEV_CRYPTO_INIT | DEV_CRYPTO_FINALIZE))
        goto pop;
      uint8_t c = 0;
      uint_fast8_t i, j;
      if (!ctx_ok)
        {
          efm32_aes_ocb_key(actx, ctx);
          c = 1;
        }
      else
        {
          /* check iv changes */
          for ((j = 16 - ctx->iv_len), (i = 0); j < 15; i++, j++)
            c |= rq->iv_ctr[i] ^ actx->ocb_nonce[j];
          c |= (rq->iv_ctr[i] ^ actx->ocb_nonce[j]) & 0xc0;
        }
      if (c)
        {
          efm32_aes_ocb_nonce(actx, rq);
        }
      efm32_aes_ocb(actx, rq, rawkey);
      break;
    }
#endif
    default:
      goto pop;
    }

 pop:
  dev_request_delayed_end(&pv->queue, rq_);
}

static DEVCRYPTO_REQUEST(efm32_aes_request)
{
  struct device_s *dev = accessor->dev;
  struct efm32_aes_private_s *pv = dev->drv_pv;

  dev_request_delayed_push(device_crypto_s_base(accessor),
                           &pv->queue, dev_crypto_rq_s_base(rq), 1);
}

static DEV_INIT(efm32_aes_init);
static DEV_CLEANUP(efm32_aes_cleanup);

#define efm32_aes_use dev_use_generic

DRIVER_DECLARE(efm32_aes_drv, 0, "EFM32 AES", efm32_aes,
               DRIVER_CRYPTO_METHODS(efm32_aes));

DRIVER_REGISTER(efm32_aes_drv);

static DEV_INIT(efm32_aes_init)
{
  struct efm32_aes_private_s *pv;

  dev->status = DEVICE_DRIVER_INIT_FAILED;

  uintptr_t addr;
  if (device_res_get_uint(dev, DEV_RES_MEM, 0, &addr, NULL))
    return -ENOENT;
  assert(addr == CONFIG_EFM32_AES_ADDR);

  pv = mem_alloc(sizeof (*pv), (mem_scope_sys));

  if (!pv)
    return -ENOMEM;
  memset(pv, 0, sizeof(*pv));

  dev->drv_pv = pv;

  dev_request_delayed_init(&pv->queue, &efm32_aes_process);

#ifdef CONFIG_DEVICE_CLOCK
  /* enable clock */
  dev_clock_sink_init(dev, &pv->clk_ep, NULL);

  if (dev_clock_sink_link(dev, &pv->clk_ep, NULL, 0, 0))
    goto err_mem;

  if (dev_clock_sink_hold(&pv->clk_ep, 0))
    goto err_clku;
#endif

  dev->drv = &efm32_aes_drv;
  dev->status = DEVICE_DRIVER_INIT_DONE;
  return 0;

 err_clk:
#ifdef CONFIG_DEVICE_CLOCK
  dev_clock_sink_release(&pv->clk_ep);
#endif
 err_clku:
#ifdef CONFIG_DEVICE_CLOCK
  dev_clock_sink_unlink(dev, &pv->clk_ep, 1);
#endif
 err_mem:
  mem_free(pv);
  return -1;
}

static DEV_CLEANUP(efm32_aes_cleanup)
{
  struct efm32_aes_private_s  *pv = dev->drv_pv;

  if (!dev_request_delayed_isidle(&pv->queue))
    return -EBUSY;

  dev_request_delayed_cleanup(&pv->queue);

#ifdef CONFIG_DEVICE_CLOCK
  dev_clock_sink_release(&pv->clk_ep);
  dev_clock_sink_unlink(dev, &pv->clk_ep, 1);
#endif

  mem_free(pv);

  return 0;
}
