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

struct soft_sha1_state_s
{
  uint32_t state[5];
  uint64_t count;
  uint8_t buffer[64];
};

struct soft_sha1_private_s
{
  struct dev_request_dlqueue_s queue;
};

static DEVCRYPTO_INFO(soft_sha1_info)
{
  memset(info, 0, sizeof(*info));
  info->name = "sha1";
  info->modes_mask = 1 << DEV_CRYPTO_MODE_HASH;
  info->cap |= DEV_CRYPTO_CAP_STATEFUL;

  info->state_size = sizeof(struct soft_sha1_state_s);
  info->block_len = 64;

  return 0;
}

static inline uint32_t rol(uint32_t value, uint_fast8_t bits)
{
  return (((value) << (bits)) | ((value) >> (32 - (bits))));
}

/* BLK0() and BLK() perform the initial expand. */
/* I got the idea of expanding during the round function from SSLeay */

#define BLK0(i) (block[i])
#define BLK(i) (block[i&15] = rol(block[(i + 13)&15] ^ block[(i + 8)&15] \
                ^ block[(i + 2)&15] ^ block[i&15], 1))

/* (R0 + R1), R2, R3, R4 are the different operations used in SHA1 */
#define R0(v, w, x, y, z, i) do { z += ((w&(x ^ y)) ^ y) + BLK0(i) + 0x5a827999 + rol(v, 5);w = rol(w, 30); } while (0)
#define R1(v, w, x, y, z, i) do { z += ((w&(x ^ y)) ^ y) + BLK(i) + 0x5a827999 + rol(v, 5);w = rol(w, 30); } while (0)
#define R2(v, w, x, y, z, i) do { z += (w ^ x ^ y) + BLK(i) + 0x6ed9eba1 + rol(v, 5);w = rol(w, 30); } while (0)
#define R3(v, w, x, y, z, i) do { z += (((w|x)&y)|(w&x)) + BLK(i) + 0x8f1bbcdc + rol(v, 5);w = rol(w, 30); } while (0)
#define R4(v, w, x, y, z, i) do { z += (w ^ x ^ y) + BLK(i) + 0xca62c1d6 + rol(v, 5);w = rol(w, 30); } while (0)


static void soft_sha1_transform(uint32_t * __restrict__ state,
                                const uint8_t * __restrict__ buffer)
{
  uint32_t a, b, c, d, e, x;
  uint32_t block[16];
  uint_fast8_t i;

  for (i = 0; i < 16; i++)
    block[i] = endian_be32_na_load(buffer + i * 4);

  a = state[0];
  b = state[1];
  c = state[2];
  d = state[3];
  e = state[4];

  for (i = 0; i <= 15; i++)
    {
      R0(a, b, c, d, e, i);
      (x = a), (a = e), (e = d), (d = c), (c = b), (b = x);
    }
  for (; i <= 19; i++)
    {
      R1(a, b, c, d, e, i);
      (x = a), (a = e), (e = d), (d = c), (c = b), (b = x);
    }
  for (; i <= 39; i++)
    {
      R2(a, b, c, d, e, i);
      (x = a), (a = e), (e = d), (d = c), (c = b), (b = x);
    }
  for (; i <= 59; i++)
    {
      R3(a, b, c, d, e, i);
      (x = a), (a = e), (e = d), (d = c), (c = b), (b = x);
    }
  for (; i <= 79; i++)
    {
      R4(a, b, c, d, e, i);
      (x = a), (a = e), (e = d), (d = c), (c = b), (b = x);
    }

  state[0] += a;
  state[1] += b;
  state[2] += c;
  state[3] += d;
  state[4] += e;
}

static void soft_sha1_update(struct soft_sha1_state_s *st,
                             const uint8_t * __restrict__ in, size_t l)
{
  uint32_t i = 0, j = st->count & 63;
  st->count += l;

  if (j + l > 63)
    {
      i = 64 - j;
      memcpy(st->buffer + j, in, i);

      soft_sha1_transform(st->state, st->buffer);
      for ( ; i + 63 < l; i += 64)
        soft_sha1_transform(st->state, in + i);
      j = 0;
    }

  memcpy(st->buffer + j, in + i, l - i);
}

static DEV_REQUEST_DELAYED_FUNC(soft_sha1_process)
{
  struct device_s *dev = accessor->dev;
  struct soft_sha1_private_s *pv = dev->drv_pv;
  struct dev_crypto_rq_s *rq = dev_crypto_rq_s_cast(rq_);
  struct dev_crypto_context_s *ctx = rq->ctx;
  struct soft_sha1_state_s *st = ctx->state_data;

  rq->err = -ENOTSUP;

  if (ctx->mode != DEV_CRYPTO_MODE_HASH)
    goto pop;

  if (rq->op & DEV_CRYPTO_INIT)
    {
      if (rq->op & DEV_CRYPTO_FINALIZE)
        st = alloca(sizeof(*st));

      st->state[0] = 0x67452301;
      st->state[1] = 0xefcdab89;
      st->state[2] = 0x98badcfe;
      st->state[3] = 0x10325476;
      st->state[4] = 0xc3d2e1f0;
      st->count = 0;
    }

  soft_sha1_update(st, rq->ad, rq->ad_len);

  if (rq->op & DEV_CRYPTO_FINALIZE)
    {
      rq->err = -EINVAL;
      if (rq->len != 20)
        goto pop;

      uint_fast8_t i;
      uint8_t count[8];

      endian_be64_na_store(count, st->count * 8);

      static const uint8_t padding[16 + 1] = { 0x80 };
      uint32_t last = st->count & 63;
      uint32_t padn = (last < 56) ? (56 - last) : (120 - last);
      const uint8_t *pad = padding;

      while (padn > 16)
        {
          soft_sha1_update(st, pad, 16);
          pad = padding + 1;
          padn -= 16;
        }

      soft_sha1_update(st, pad, padn);
      soft_sha1_update(st, count, 8);

      for (i = 0; i < 5; i++)
        endian_be32_na_store(rq->out + i * 4, st->state[i]);
    }

  rq->err = 0;

 pop:
  dev_request_delayed_end(&pv->queue, rq_);
}

static DEVCRYPTO_REQUEST(soft_sha1_request)
{
  struct device_s *dev = accessor->dev;
  struct soft_sha1_private_s *pv = dev->drv_pv;

  dev_request_delayed_push(device_crypto_s_base(accessor),
                           &pv->queue, dev_crypto_rq_s_base(rq), 0);
}

static DEV_INIT(soft_sha1_init);
static DEV_CLEANUP(soft_sha1_cleanup);

#define soft_sha1_use dev_use_generic

DRIVER_DECLARE(soft_sha1_drv, 0, "Software SHA1 hash", soft_sha1,
               DRIVER_CRYPTO_METHODS(soft_sha1));

DRIVER_REGISTER(soft_sha1_drv);

DEV_DECLARE_STATIC(soft_sha1_dev, "sha1_soft", 0, soft_sha1_drv);

static DEV_INIT(soft_sha1_init)
{
  struct soft_sha1_private_s *pv;


  pv = mem_alloc(sizeof (*pv), (mem_scope_sys));

  if (!pv)
    return -ENOMEM;
  memset(pv, 0, sizeof(*pv));

  dev->drv_pv = pv;

  dev_request_delayed_init(&pv->queue, &soft_sha1_process);

  return 0;

err_mem:
  mem_free(pv);
  return -1;
}

static DEV_CLEANUP(soft_sha1_cleanup)
{
  struct soft_sha1_private_s  *pv = dev->drv_pv;

  if (!dev_request_delayed_isidle(&pv->queue))
    return -EBUSY;

  dev_request_delayed_cleanup(&pv->queue);

  mem_free(pv);

  return 0;
}
