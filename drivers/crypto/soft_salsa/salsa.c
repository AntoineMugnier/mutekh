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

    Copyright Alexandre Becoulet <alexandre.becoulet@free.fr> (c) 2015

*/

#include <hexo/types.h>
#include <hexo/endian.h>

#include <mutek/mem_alloc.h>
#include <device/request.h>
#include <device/device.h>
#include <device/resources.h>
#include <device/driver.h>
#include <device/class/crypto.h>

typedef uint32_t soft_salsa_state_t[12];

struct soft_salsa_private_s
{
  struct dev_request_dlqueue_s queue;
};

static DEVCRYPTO_INFO(soft_salsa_info)
{
  memset(info, 0, sizeof(*info));

  switch (accessor->number)
    {
#ifdef CONFIG_DRIVER_CRYPTO_SOFT_SALSA20
    case 0:
      info->name = "salsa20";
      break;
#endif
#ifdef CONFIG_DRIVER_CRYPTO_SOFT_CHACHA20
    case 1:
      info->name = "chacha20";
      break;
#endif
    default:
      return -ENOENT;
    }

  info->modes_mask = 0
#ifdef CONFIG_DRIVER_CRYPTO_SOFT_SALSA_STREAM
    | (1 << DEV_CRYPTO_MODE_STREAM)
#endif
#ifdef CONFIG_DRIVER_CRYPTO_SOFT_SALSA_RANDOM
    | (1 << DEV_CRYPTO_MODE_RANDOM)
#endif
    ;

  info->cap |= DEV_CRYPTO_CAP_INPLACE | DEV_CRYPTO_CAP_NOTINPLACE
    | DEV_CRYPTO_CAP_STATEFUL
    | DEV_CRYPTO_CAP_128BITS_KEY | DEV_CRYPTO_CAP_256BITS_KEY;

  info->state_size = sizeof(soft_salsa_state_t);

  return 0;
}

static inline uint32_t salsa_rotate(uint32_t x, uint_fast8_t i)
{
  return (x << i) | (x >> (32 - i));
}

#define SALSA_CIPHER_FUNC(n) void (n)(uint32_t out[16], const uint32_t in[12], bool_t k16)

typedef SALSA_CIPHER_FUNC(salsa_cipher_t);

#ifdef CONFIG_DRIVER_CRYPTO_SOFT_CHACHA20
static SALSA_CIPHER_FUNC(chacha20_cipher)
{
  uint_fast8_t i;

  uint32_t c0 = 0x61707865;
  uint32_t c15 = 0x6b206574;
  uint32_t c5, c10;
#ifdef CONFIG_DRIVER_CRYPTO_SOFT_SALSA_STREAM
  if (k16)
    {
      c5 = 0x3120646e;
      c10 = 0x79622d36;
    }
  else
#endif
    {
      c5 = 0x3320646e;
      c10 = 0x79622d32;
    }

  uint32_t x0  = c0;
  uint32_t x1  = c5;
  uint32_t x2  = c10;
  uint32_t x3  = c15;
  uint32_t x4  = in[0];
  uint32_t x5  = in[1];
  uint32_t x6  = in[2];
  uint32_t x7  = in[3];
  uint32_t x8  = in[8];
  uint32_t x9  = in[9];
  uint32_t x10 = in[10];
  uint32_t x11 = in[11];
  uint32_t x12 = in[6];
  uint32_t x13 = in[7];
  uint32_t x14 = in[4];
  uint32_t x15 = in[5];

#define CAHCHA_QUARTERROUND(a,b,c,d)                 \
  x##a += x##b;                                      \
  x##d = salsa_rotate(x##d ^ x##a, 16);              \
  x##c += x##d;                                      \
  x##b = salsa_rotate(x##b ^ x##c, 12);              \
  x##a += x##b;                                      \
  x##d = salsa_rotate(x##d ^ x##a, 8);               \
  x##c += x##d;                                      \
  x##b = salsa_rotate(x##b ^ x##c, 7);

  for (i = 20; i > 0;i -= 2)
    {
      CAHCHA_QUARTERROUND(0, 4,  8, 12);
      CAHCHA_QUARTERROUND(1, 5,  9, 13);
      CAHCHA_QUARTERROUND(2, 6, 10, 14);
      CAHCHA_QUARTERROUND(3, 7, 11, 15);
      CAHCHA_QUARTERROUND(0, 5, 10, 15);
      CAHCHA_QUARTERROUND(1, 6, 11, 12);
      CAHCHA_QUARTERROUND(2, 7,  8, 13);
      CAHCHA_QUARTERROUND(3, 4,  9, 14);
    }

  out[0 ] = x0  + c0;
  out[1 ] = x1  + c5;
  out[2 ] = x2  + c10;
  out[3 ] = x3  + c15;
  out[4 ] = x4  + in[0];
  out[5 ] = x5  + in[1];
  out[6 ] = x6  + in[2];
  out[7 ] = x7  + in[3];
  out[8 ] = x8  + in[8];
  out[9 ] = x9  + in[9];
  out[10] = x10 + in[10];
  out[11] = x11 + in[11];
  out[12] = x12 + in[6];
  out[13] = x13 + in[7];
  out[14] = x14 + in[4];
  out[15] = x15 + in[5];
}
#endif

#ifdef CONFIG_DRIVER_CRYPTO_SOFT_SALSA20
static SALSA_CIPHER_FUNC(salsa20_cipher)
{
  uint_fast8_t i;

  uint32_t c0 = 0x61707865;
  uint32_t c15 = 0x6b206574;
  uint32_t c5, c10;
#ifdef CONFIG_DRIVER_CRYPTO_SOFT_SALSA_STREAM
  if (k16)
    {
      c5 = 0x3120646e;
      c10 = 0x79622d36;
    }
  else
#endif
    {
      c5 = 0x3320646e;
      c10 = 0x79622d32;
    }

  uint32_t x0 = c0;
  uint32_t x1 = in[0];
  uint32_t x2 = in[1];
  uint32_t x3 = in[2];
  uint32_t x4 = in[3];
  uint32_t x5 = c5;
  uint32_t x6 = in[4];
  uint32_t x7 = in[5];
  uint32_t x8 = in[6];
  uint32_t x9 = in[7];
  uint32_t x10 = c10;
  uint32_t x11 = in[8];
  uint32_t x12 = in[9];
  uint32_t x13 = in[10];
  uint32_t x14 = in[11];
  uint32_t x15 = c15;

  for (i = 20; i > 0; i -= 2)
    {
      x4 =   x4 ^ salsa_rotate( x0 + x12,  7);
      x8 =   x8 ^ salsa_rotate( x4 +  x0,  9);
      x12 = x12 ^ salsa_rotate( x8 +  x4, 13);
      x0 =   x0 ^ salsa_rotate(x12 +  x8, 18);
      x9 =   x9 ^ salsa_rotate( x5 +  x1,  7);
      x13 = x13 ^ salsa_rotate( x9 +  x5,  9);
      x1 =   x1 ^ salsa_rotate(x13 +  x9, 13);
      x5 =   x5 ^ salsa_rotate( x1 + x13, 18);
      x14 = x14 ^ salsa_rotate(x10 +  x6,  7);
      x2 =   x2 ^ salsa_rotate(x14 + x10,  9);
      x6 =   x6 ^ salsa_rotate( x2 + x14, 13);
      x10 = x10 ^ salsa_rotate( x6 +  x2, 18);
      x3 =   x3 ^ salsa_rotate(x15 + x11,  7);
      x7 =   x7 ^ salsa_rotate( x3 + x15,  9);
      x11 = x11 ^ salsa_rotate( x7 +  x3, 13);
      x15 = x15 ^ salsa_rotate(x11 +  x7, 18);
      x1 =   x1 ^ salsa_rotate( x0 +  x3,  7);
      x2 =   x2 ^ salsa_rotate( x1 +  x0,  9);
      x3 =   x3 ^ salsa_rotate( x2 +  x1, 13);
      x0 =   x0 ^ salsa_rotate( x3 +  x2, 18);
      x6 =   x6 ^ salsa_rotate( x5 +  x4,  7);
      x7 =   x7 ^ salsa_rotate( x6 +  x5,  9);
      x4 =   x4 ^ salsa_rotate( x7 +  x6, 13);
      x5 =   x5 ^ salsa_rotate( x4 +  x7, 18);
      x11 = x11 ^ salsa_rotate(x10 +  x9,  7);
      x8 =   x8 ^ salsa_rotate(x11 + x10,  9);
      x9 =   x9 ^ salsa_rotate( x8 + x11, 13);
      x10 = x10 ^ salsa_rotate( x9 +  x8, 18);
      x12 = x12 ^ salsa_rotate(x15 + x14,  7);
      x13 = x13 ^ salsa_rotate(x12 + x15,  9);
      x14 = x14 ^ salsa_rotate(x13 + x12, 13);
      x15 = x15 ^ salsa_rotate(x14 + x13, 18);
    }

  out[0 ] = x0  + c0;
  out[1 ] = x1  + in[0];
  out[2 ] = x2  + in[1];
  out[3 ] = x3  + in[2];
  out[4 ] = x4  + in[3];
  out[5 ] = x5  + c5;
  out[6 ] = x6  + in[4];
  out[7 ] = x7  + in[5];
  out[8 ] = x8  + in[6];
  out[9 ] = x9  + in[7];
  out[10] = x10 + c10;
  out[11] = x11 + in[8];
  out[12] = x12 + in[9];
  out[13] = x13 + in[10];
  out[14] = x14 + in[11];
  out[15] = x15 + c15;
}
#endif

static DEV_REQUEST_DELAYED_FUNC(soft_salsa_process)
{
  struct device_s *dev = accessor->dev;
  struct soft_salsa_private_s *pv = dev->drv_pv;
  struct dev_crypto_rq_s *rq = dev_crypto_rq_s_cast(rq_);
  struct dev_crypto_context_s *ctx = rq->ctx;
  uint32_t * __restrict__ st = ctx->state_data;
  salsa_cipher_t *cipher;

  rq->err = -ENOTSUP;

  switch (accessor->number)
    {
#ifdef CONFIG_DRIVER_CRYPTO_SOFT_SALSA20
    case 0:
      cipher = &salsa20_cipher;
      break;
#endif
#ifdef CONFIG_DRIVER_CRYPTO_SOFT_CHACHA20
    case 1:
      cipher = &chacha20_cipher;
      break;
#endif
    default:
      goto pop;
    }

  switch (ctx->mode)
    {
#ifdef CONFIG_DRIVER_CRYPTO_SOFT_SALSA_STREAM
    case DEV_CRYPTO_MODE_STREAM: {
      if (rq->op & DEV_CRYPTO_INIT)
        {
          if (rq->op & DEV_CRYPTO_FINALIZE)
            st = alloca(sizeof(soft_salsa_state_t));
          else if (ctx->cache_ptr == st)
            goto done;
          else
            ctx->cache_ptr = st;

          const uint8_t * __restrict__ k = ctx->key_data;
          st[0] = endian_le32_na_load(k + 0);
          st[1] = endian_le32_na_load(k + 4);
          st[2] = endian_le32_na_load(k + 8);
          st[3] = endian_le32_na_load(k + 12);
          switch (ctx->key_len)
            {
            case 32:
              k += 16;
            case 16:
              break;
            default:
              goto pop;
            }
          st[8] = endian_le32_na_load(k + 0);
          st[9] = endian_le32_na_load(k + 4);
          st[10] = endian_le32_na_load(k + 8);
          st[11] = endian_le32_na_load(k + 12);
        done:

          st[6] = st[7] = 0;
          switch (ctx->iv_len)
            {
              /* no IV provided, IV=0 */
            case 0:
              st[4] = st[5] = 0;
              break;
            case 16:
              /* IV and 64 bits stream byte offset provided */
              st[7] = endian_le32_na_load(rq->iv_ctr + 12);
            case 12:
              /* IV and 32 bits stream byte offset provided */
              st[6] = endian_le32_na_load(rq->iv_ctr + 8);
            case 8:
              /* IV provided */
              st[5] = endian_le32_na_load(rq->iv_ctr + 4);
              st[4] = endian_le32_na_load(rq->iv_ctr + 0);
              break;
            default:
              goto pop;
            }
        }

      size_t l = rq->len;
      const uint8_t * __restrict__ in = rq->in;
      uint8_t * __restrict__ out = rq->out;
      uint64_t offset = ((uint64_t)st[7] << 32) | st[6];
      while (l)
        {
          size_t r = __MIN(l, 64 - (offset & 63));
          if (out)
            {
              uint32_t stream[16];
              st[7] = offset >> 38;
              st[6] = offset >> 6;
              cipher(stream, st, ctx->key_len == 16);
              uint_fast8_t i, o = offset & 63;
              for (i = o; i < o + r; i++)
                {
                  uint8_t s = stream[i >> 2] >> ((i & 3) * 8);
                  if (in)
                    *out++ = *in++ ^ s;
                  else
                    *out++ = s;
                }
            }
          offset += r;
          l -= r;
        }
      st[7] = offset >> 32;
      st[6] = offset;

      rq->err = 0;
      break;
    }
#endif

#ifdef CONFIG_DRIVER_CRYPTO_SOFT_SALSA_RANDOM
    case DEV_CRYPTO_MODE_RANDOM: {

      if (rq->op & DEV_CRYPTO_INIT)
        memset(st, 0, sizeof(soft_salsa_state_t));

      size_t l = rq->ad_len;
      if ((rq->op & DEV_CRYPTO_INVERSE) && l)
        {
          const uint8_t * __restrict__ in = rq->ad;
          uint8_t *r8 = (uint8_t*)st;
          while (l--)
            {
              uint_fast8_t i = l % 32;
              r8[i] ^= *in++;
              if (!i)
                {
                  uint32_t stream[16];
                  cipher(stream, st, 0);
                  memcpy(st, stream, sizeof(soft_salsa_state_t));
                }
            }
        }

      l = rq->len;
      if ((rq->op & DEV_CRYPTO_FINALIZE) && l)
        {
          union {
            uint32_t r32[16];
            uint8_t  r8[64];
          }          rout;
          uint8_t * __restrict__ out = rq->out;
          memcpy(rout.r32, st, sizeof(soft_salsa_state_t));
          while (l)
            {
              cipher(rout.r32, rout.r32, 0);
              size_t r = __MIN(l, 16);
              l -= r;
              while (r--)
                *out++ = rout.r8[48 + r];
            }
          memcpy(st, rout.r32, sizeof(soft_salsa_state_t));
        }
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

static DEVCRYPTO_REQUEST(soft_salsa_request)
{
  struct device_s *dev = accessor->dev;
  struct soft_salsa_private_s *pv = dev->drv_pv;

  dev_request_delayed_push(device_crypto_s_base(accessor),
                           &pv->queue, dev_crypto_rq_s_base(rq), 0);
}

static DEV_INIT(soft_salsa_init);
static DEV_CLEANUP(soft_salsa_cleanup);

#define soft_salsa_use dev_use_generic

DRIVER_DECLARE(soft_salsa_drv, 0, "Software Salsa family of ciphers", soft_salsa,
               DRIVER_CRYPTO_METHODS(soft_salsa));

DRIVER_REGISTER(soft_salsa_drv);

DEV_DECLARE_STATIC(soft_salsa_dev, "salsa_soft", 0, soft_salsa_drv);

static DEV_INIT(soft_salsa_init)
{
  struct soft_salsa_private_s *pv;

  dev->status = DEVICE_DRIVER_INIT_FAILED;

  pv = mem_alloc(sizeof (*pv), (mem_scope_sys));

  if (!pv)
    return -ENOMEM;
  memset(pv, 0, sizeof(*pv));

  dev->drv_pv = pv;

  dev_request_delayed_init(&pv->queue, &soft_salsa_process);

  dev->drv = &soft_salsa_drv;
  dev->status = DEVICE_DRIVER_INIT_DONE;
  return 0;

err_mem:
  mem_free(pv);
  return -1;
}

static DEV_CLEANUP(soft_salsa_cleanup)
{
  struct soft_salsa_private_s  *pv = dev->drv_pv;

  mem_free(pv);
}
