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

    Copyright Alexandre Becoulet <alexandre.becoulet@free.fr> (c) 2014

*/

#include "aes.h"

#include <mutek/mem_alloc.h>
#include <mutek/kroutine.h>

static DEVCRYPTO_INFO(soft_aes_info)
{
  if (accessor->number > 0)
    return -ENOENT;

  memset(info, 0, sizeof(*info));
  info->name = "aes";
  info->modes_mask = 0
#ifdef CONFIG_DRIVER_CRYPTO_SOFT_AES_ECB
    | (1 << DEV_CRYPTO_MODE_ECB)
#endif
#ifdef CONFIG_DRIVER_CRYPTO_SOFT_AES_CBC
    | (1 << DEV_CRYPTO_MODE_CBC)
#endif
#ifdef CONFIG_DRIVER_CRYPTO_SOFT_AES_CTR
    | (1 << DEV_CRYPTO_MODE_CTR)
#endif
#ifdef CONFIG_DRIVER_CRYPTO_SOFT_AES_OCB3
    | (1 << DEV_CRYPTO_MODE_OCB3)
#endif
    ;

  info->cap |= DEV_CRYPTO_CAP_INPLACE | DEV_CRYPTO_CAP_NOTINPLACE
    | DEV_CRYPTO_CAP_128BITS_KEY | DEV_CRYPTO_CAP_STATEFUL
#ifdef CONFIG_DRIVER_EFM32_AES_AD
    | DEV_CRYPTO_CAP_ASSOC_DATA
#endif
#ifdef CONFIG_DRIVER_CRYPTO_SOFT_AES192
    | DEV_CRYPTO_CAP_192BITS_KEY
#endif
#ifdef CONFIG_DRIVER_CRYPTO_SOFT_AES256
    | DEV_CRYPTO_CAP_256BITS_KEY
#endif
    ;

  info->state_size = sizeof(struct soft_aes_state_s);

  info->block_len = 16;

  return 0;
};

static DEV_REQUEST_DELAYED_FUNC(soft_aes_process)
{
  struct device_s *dev = accessor->dev;
  struct soft_aes_private_s *pv = dev->drv_pv;
  struct dev_crypto_rq_s *rq = dev_crypto_rq_s_cast(rq_);
  struct dev_crypto_context_s *ctx = rq->ctx;

  bool_t ctx_ok = dev_crypto_ctx_bind(ctx, pv->ctx, &pv->next,
                            CONFIG_DRIVER_CRYPTO_SOFT_AES_CTXCOUNT);
  struct soft_aes_context_s *actx = pv->actx + ctx->cache_id;

  rq->err = -ENOTSUP;

  if (!ctx_ok)
    {
      uint_fast8_t i, r;

      if (ctx->key_len > 32)
        goto pop;

      for (i = 0; i < ctx->key_len / 4; i++)
        actx->key[i] = endian_be32_na_load(ctx->key_data + i * 4);

      switch (ctx->key_len)
        {
        case 16:
          soft_aes_expand_key(actx->rkey[0], 4, r = 11);
          break;
#ifdef CONFIG_DRIVER_CRYPTO_SOFT_AES192
        case 24:
          soft_aes_expand_key(actx->rkey[0], 6, r = 13);
          break;
#endif
#ifdef CONFIG_DRIVER_CRYPTO_SOFT_AES256
        case 32:
          soft_aes_expand_key(actx->rkey[0], 8, r = 15);
          break;
#endif
        default:
          goto pop;
        }

      actx->rounds = r;
    }

  switch (ctx->mode)
    {
#ifdef CONFIG_DRIVER_CRYPTO_SOFT_AES_ECB
    case DEV_CRYPTO_MODE_ECB:
      soft_aes_ecb(actx, rq);
      break;
#endif
#ifdef CONFIG_DRIVER_CRYPTO_SOFT_AES_CBC
    case DEV_CRYPTO_MODE_CBC:
      soft_aes_cbc(actx, rq);
      break;
#endif
#ifdef CONFIG_DRIVER_CRYPTO_SOFT_AES_CTR
    case DEV_CRYPTO_MODE_CTR:
      soft_aes_ctr(actx, rq);
      break;
#endif
#ifdef CONFIG_DRIVER_CRYPTO_SOFT_AES_OCB3
    case DEV_CRYPTO_MODE_OCB3:
      soft_aes_ocb(actx, rq, ctx_ok);
      break;
#endif
    default:
      break;
    }

 pop:
  dev_request_delayed_end(&pv->queue, rq_);
}

static DEVCRYPTO_REQUEST(soft_aes_request)
{
  struct device_s *dev = accessor->dev;
  struct soft_aes_private_s *pv = dev->drv_pv;

  dev_request_delayed_push(device_crypto_s_base(accessor),
                           &pv->queue, dev_crypto_rq_s_base(rq), 0);
}

static DEV_INIT(soft_aes_init);
static DEV_CLEANUP(soft_aes_cleanup);

const struct driver_s soft_aes_drv =
{
  .desc       = "Software AES cipher",
  .f_init     = soft_aes_init,
  .f_cleanup  = soft_aes_cleanup,
  .classes    = {
    DRIVER_CRYPTO_METHODS(soft_aes),
    0,
  },
};

REGISTER_DRIVER(soft_aes_drv);

DEV_DECLARE_STATIC(soft_aes_dev, "aes_soft", 0, soft_aes_drv);

static DEV_INIT(soft_aes_init)
{
  struct soft_aes_private_s *pv;

  dev->status = DEVICE_DRIVER_INIT_FAILED;

  pv = mem_alloc(sizeof (*pv), (mem_scope_sys));

  if (!pv)
    return -ENOMEM;
  memset(pv, 0, sizeof(*pv));

  dev->drv_pv = pv;

  dev_request_delayed_init(&pv->queue, &soft_aes_process);

  dev->drv = &soft_aes_drv;
  dev->status = DEVICE_DRIVER_INIT_DONE;
  return 0;

 err_mem:
  mem_free(pv);
  return -1;
}

static DEV_CLEANUP(soft_aes_cleanup)
{
  struct soft_aes_private_s  *pv = dev->drv_pv;

  mem_free(pv);
}

