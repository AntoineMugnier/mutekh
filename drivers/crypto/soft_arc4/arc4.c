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

#include <hexo/types.h>
#include <hexo/endian.h>

#include <mutek/mem_alloc.h>
#include <device/request.h>
#include <device/device.h>
#include <device/resources.h>
#include <device/driver.h>
#include <device/class/crypto.h>

struct soft_arc4_state_s
{
  uint8_t state[256];
  uint8_t a, b;
};

struct soft_arc4_private_s
{
  struct dev_request_dlqueue_s queue;
};

static DEVCRYPTO_INFO(soft_arc4_info)
{
  if (accessor->number > 0)
    return -ENOENT;

  memset(info, 0, sizeof(*info));
  info->name = "arc4";
  info->modes_mask = 0
#ifdef CONFIG_DRIVER_CRYPTO_SOFT_ARC4_STREAM
    | (1 << DEV_CRYPTO_MODE_STREAM)
#endif
#ifdef CONFIG_DRIVER_CRYPTO_SOFT_ARC4_RANDOM
    | (1 << DEV_CRYPTO_MODE_RANDOM)
#endif
    ;
  info->cap |= DEV_CRYPTO_CAP_INPLACE | DEV_CRYPTO_CAP_NOTINPLACE
    | DEV_CRYPTO_CAP_STATEFUL
    | DEV_CRYPTO_CAP_128BITS_KEY | DEV_CRYPTO_CAP_192BITS_KEY
    | DEV_CRYPTO_CAP_256BITS_KEY | DEV_CRYPTO_CAP_VARLEN_KEY;

  info->state_size = sizeof(struct soft_arc4_state_s);

  return 0;
}

static void soft_arc4_stinit(struct soft_arc4_state_s *st)
{
  uint_fast16_t a;
  uint8_t * __restrict__ state = st->state;

  for (a = 0; a < 256; a++)
    state[a] = a;
  st->a = st->b = 0;
}

static void soft_arc4_key(struct soft_arc4_state_s *st,
                          const uint8_t * __restrict__ key, size_t l)
{
  uint8_t * __restrict__ state = st->state;
  uint_fast16_t a, b, k;

  for (k = a = b = 0; a < 256; a++, k++)
    {
      uint8_t tmp;

      k &= (l == k) - 1;
      tmp = state[a];
      b = (uint8_t)(b + tmp + key[k]);

      state[a] = state[b];
      state[b] = tmp;
    }
}

static void soft_arc4_stream(struct soft_arc4_state_s *st,
                             const uint8_t * __restrict__ in,
                             uint8_t * __restrict__ out, size_t l)
{
  uint_fast8_t a = st->a, b = st->b;
  uint8_t * __restrict__ state = st->state;

  while (l--)
    {
      uint8_t tmp;
      a = (uint8_t)(a + 1);
      b = (uint8_t)(b + state[a]);

      tmp = state[b];
      state[b] = state[a];
      state[a] = tmp;

      if (out)
        {
          if (in)
            *out++ = *in++ ^ state[(uint8_t)(state[a] + state[b])];
          else
            *out++ = state[(uint8_t)(state[a] + state[b])];
        }
    }

  st->a = a;
  st->b = b;
}

static DEV_REQUEST_DELAYED_FUNC(soft_arc4_process)
{
  struct device_s *dev = accessor->dev;
  struct soft_arc4_private_s *pv = dev->drv_pv;
  struct dev_crypto_rq_s *rq = dev_crypto_rq_s_cast(rq_);
  struct dev_crypto_context_s *ctx = rq->ctx;
  struct soft_arc4_state_s *st = ctx->state_data;

  rq->err = -ENOTSUP;

  switch (ctx->mode)
    {
#ifdef CONFIG_DRIVER_CRYPTO_SOFT_ARC4_STREAM
    case DEV_CRYPTO_MODE_STREAM: {
      if (rq->op & DEV_CRYPTO_INIT)
        {
          if (rq->op & DEV_CRYPTO_FINALIZE)
            st = alloca(sizeof(*st));
          else if (ctx->cache_ptr == st)
            goto done;
          else
            ctx->cache_ptr = st;

          if (ctx->key_len > 256 || ctx->iv_len > 0)
            goto pop;

          soft_arc4_stinit(st);
          soft_arc4_key(st, ctx->key_data, ctx->key_len);
        done:;
        }

      soft_arc4_stream(st, rq->in, rq->out, rq->len);
      rq->err = 0;
      break;
    }
#endif

#ifdef CONFIG_DRIVER_CRYPTO_SOFT_ARC4_RANDOM
    case DEV_CRYPTO_MODE_RANDOM: {
      if (rq->op & DEV_CRYPTO_INIT)
        soft_arc4_stinit(st);
      if (rq->op & DEV_CRYPTO_INVERSE)
        {
          size_t l = rq->ad_len;
          const uint8_t *in = rq->ad;
          while (1)
            {
              soft_arc4_key(st, in, l);
              if (l <= 256)
                break;
              l -= 256;
              in += 256;
            }
          soft_arc4_stream(st, NULL, NULL, 768);
        }
      if (rq->op & DEV_CRYPTO_FINALIZE)
        soft_arc4_stream(st, NULL, rq->out, rq->len);
      rq->err = 0;
      break;
    }
#endif

    default:
      break;
    }

 pop:
  dev_request_delayed_end(&pv->queue, rq_);
}

static DEVCRYPTO_REQUEST(soft_arc4_request)
{
  struct device_s *dev = accessor->dev;
  struct soft_arc4_private_s *pv = dev->drv_pv;

  dev_request_delayed_push(device_crypto_s_base(accessor),
                           &pv->queue, dev_crypto_rq_s_base(rq), 0);
}

static const struct driver_crypto_s soft_arc4_crypto_drv =
  {
    .class_         = DRIVER_CLASS_CRYPTO,
    .f_info         = soft_arc4_info,
    .f_request      = soft_arc4_request,
  };

static DEV_INIT(soft_arc4_init);
static DEV_CLEANUP(soft_arc4_cleanup);

const struct driver_s soft_arc4_drv =
  {
    .desc       = "Software ARC4 cipher",
    .f_init     = soft_arc4_init,
    .f_cleanup  = soft_arc4_cleanup,
    .classes    = {
      &soft_arc4_crypto_drv,
      NULL
    }
  };

REGISTER_DRIVER(soft_arc4_drv);

DEV_DECLARE_STATIC(soft_arc4_dev, "arc4_soft", 0, soft_arc4_drv, NULL);

static DEV_INIT(soft_arc4_init)
{
  struct soft_arc4_private_s *pv;

  dev->status = DEVICE_DRIVER_INIT_FAILED;

  pv = mem_alloc(sizeof (*pv), (mem_scope_sys));

  if (!pv)
    return -ENOMEM;
  memset(pv, 0, sizeof(*pv));

  dev->drv_pv = pv;

  dev_request_delayed_init(&pv->queue, &soft_arc4_process);

  dev->drv = &soft_arc4_drv;
  dev->status = DEVICE_DRIVER_INIT_DONE;
  return 0;

err_mem:
  mem_free(pv);
  return -1;
}

static DEV_CLEANUP(soft_arc4_cleanup)
{
  struct soft_arc4_private_s  *pv = dev->drv_pv;

  mem_free(pv);
}
