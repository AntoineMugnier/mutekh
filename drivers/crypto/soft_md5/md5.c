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

struct soft_md5_state_s
{
  uint32_t state[4];
  uint64_t count;
  uint8_t buffer[64];
};

struct soft_md5_private_s
{
  struct dev_request_dlqueue_s queue;
};

static DEVCRYPTO_INFO(soft_md5_info)
{
  if (accessor->number > 0)
    return -ENOENT;

  memset(info, 0, sizeof(*info));
  info->name = "md5";
  info->modes_mask = 1 << DEV_CRYPTO_MODE_HASH;
  info->cap |= DEV_CRYPTO_CAP_STATEFUL;

  info->state_size = sizeof(struct soft_md5_state_s);
  info->block_len = 64;

  return 0;
}

#define MD5F1(x, y, z) (z ^ (x & (y ^ z)))
#define MD5F2(x, y, z) MD5F1(z, x, y)
#define MD5F3(x, y, z) (x ^ y ^ z)
#define MD5F4(x, y, z) (y ^ (x | ~z))

#define MD5STEP(f, w, x, y, z, data, s)		\
  do {						\
    w += MD5##f(x, y, z) + data;		\
    w = w<<s | w>>(32-s);			\
    w += x;					\
  } while (0)

static void
soft_md5_transform(uint32_t * __restrict__ state, const uint8_t * __restrict__ block)
{
  uint32_t a = state[0];
  uint32_t b = state[1];
  uint32_t c = state[2];
  uint32_t d = state[3];
  uint32_t x;
  uint_fast8_t i;

  static const uint32_t key[64] = {
    0xd76aa478, 0xe8c7b756, 0x242070db, 0xc1bdceee, 0xf57c0faf, 0x4787c62a, 0xa8304613, 0xfd469501,
    0x698098d8, 0x8b44f7af, 0xffff5bb1, 0x895cd7be, 0x6b901122, 0xfd987193, 0xa679438e, 0x49b40821,
    0xf61e2562, 0xc040b340, 0x265e5a51, 0xe9b6c7aa, 0xd62f105d, 0x02441453, 0xd8a1e681, 0xe7d3fbc8,
    0x21e1cde6, 0xc33707d6, 0xf4d50d87, 0x455a14ed, 0xa9e3e905, 0xfcefa3f8, 0x676f02d9, 0x8d2a4c8a,
    0xfffa3942, 0x8771f681, 0x6d9d6122, 0xfde5380c, 0xa4beea44, 0x4bdecfa9, 0xf6bb4b60, 0xbebfbc70,
    0x289b7ec6, 0xeaa127fa, 0xd4ef3085, 0x04881d05, 0xd9d4d039, 0xe6db99e5, 0x1fa27cf8, 0xc4ac5665,
    0xf4292244, 0x432aff97, 0xab9423a7, 0xfc93a039, 0x655b59c3, 0x8f0ccc92, 0xffeff47d, 0x85845dd1,
    0x6fa87e4f, 0xfe2ce6e0, 0xa3014314, 0x4e0811a1, 0xf7537e82, 0xbd3af235, 0x2ad7d2bb, 0xeb86d391,
  };

  for (i = 0; i < 16; i++)
    {
      MD5STEP(F1, a, b, c, d, endian_le32_na_load(block + i * 4) + key[i],
	      ((0x16110c07 >> ((i & 3) * 8)) & 0xff));
      (x = a), (a = d), (d = c), (c = b), (b = x);
    }

  for (; i < 32; i++)
    {
      MD5STEP(F2, a, b, c, d, endian_le32_na_load(block + (20 * i + 4) % 64) + key[i],
	      ((0x140e0905 >> ((i & 3) * 8)) & 0xff));
      (x = a), (a = d), (d = c), (c = b), (b = x);
    }

  for (; i < 48; i++)
    {
      MD5STEP(F3, a, b, c, d, endian_le32_na_load(block + (12 * i + 20) % 64) + key[i],
	      ((0x17100b04 >> ((i & 3) * 8)) & 0xff));
      (x = a), (a = d), (d = c), (c = b), (b = x);
    }

  for (; i < 64; i++)
    {
      MD5STEP(F4, a, b, c, d, endian_le32_na_load(block + (28 * i) % 64) + key[i],
	      ((0x150f0a06 >> ((i & 3) * 8)) & 0xff));
      (x = a), (a = d), (d = c), (c = b), (b = x);
    }

  state[0] += a;
  state[1] += b;
  state[2] += c;
  state[3] += d;
}

static void soft_md5_update(struct soft_md5_state_s *st,
                            const uint8_t * __restrict__ data, size_t len)
{
  size_t have = (st->count / 8) % 64;
  size_t need = 64 - have;

  st->count += len * 8;

  if (len >= need)
    {
      /* Append to data from a previous call and transform 1 block. */
      if (have != 0)
	{
	  memcpy(st->buffer + have, data, need);
	  soft_md5_transform(st->state, st->buffer);
	  data += need;
	  len -= need;
	  have = 0;
	}

      /* Process data in 64 bytes chunks. */
      while (len >= 64)
	{
	  soft_md5_transform(st->state, data);
	  data += 64;
	  len -= 64;
	}
    }

  /* Keep any remaining bytes of data for next call. */
  if (len != 0)
    memcpy(st->buffer + have, data, len);
}

static DEV_REQUEST_DELAYED_FUNC(soft_md5_process)
{
  struct device_s *dev = accessor->dev;
  struct soft_md5_private_s *pv = dev->drv_pv;
  struct dev_crypto_rq_s *rq = dev_crypto_rq_s_cast(rq_);
  struct dev_crypto_context_s *ctx = rq->ctx;
  struct soft_md5_state_s *st = ctx->state_data;

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
      st->count = 0;
    }

  soft_md5_update(st, rq->ad, rq->ad_len);

  if (rq->op & DEV_CRYPTO_FINALIZE)
    {
      rq->err = -EINVAL;
      if (rq->len != 16)
        goto pop;

      uint8_t count[8];
      size_t padlen;
      static const uint8_t padding[16 + 1] = { 0x80 };

      endian_le64_na_store(count, st->count);

      padlen = 64 - (st->count / 8) % 64;
      if (padlen < 1 + 8)
        padlen += 64;
      padlen -= 8;

      const uint8_t *pad = padding;
      while (padlen > 16)
        {
          soft_md5_update(st, pad, 16);
          pad = padding + 1;
          padlen -= 16;
        }
      soft_md5_update(st, pad, padlen);
      soft_md5_update(st, count, 8);

      uint_fast8_t i;
      for (i = 0; i < 5; i++)
        endian_le32_na_store(rq->out + i * 4, st->state[i]);
    }
  
  rq->err = 0;

 pop:
  dev_request_delayed_end(&pv->queue, rq_);
}

static DEVCRYPTO_REQUEST(soft_md5_request)
{
  struct device_s *dev = accessor->dev;
  struct soft_md5_private_s *pv = dev->drv_pv;

  dev_request_delayed_push(device_crypto_s_base(accessor),
                           &pv->queue, dev_crypto_rq_s_base(rq), 0);
}

static const struct driver_crypto_s soft_md5_crypto_drv =
  {
    .class_         = DRIVER_CLASS_CRYPTO,
    .f_info         = soft_md5_info,
    .f_request      = soft_md5_request,
  };

static DEV_INIT(soft_md5_init);
static DEV_CLEANUP(soft_md5_cleanup);

const struct driver_s soft_md5_drv =
  {
    .desc       = "Software MD5 hash",
    .f_init     = soft_md5_init,
    .f_cleanup  = soft_md5_cleanup,
    .classes    = {
      &soft_md5_crypto_drv,
      NULL
    }
  };

REGISTER_DRIVER(soft_md5_drv);

DEV_DECLARE_STATIC(soft_md5_dev, "md5_soft", 0, soft_md5_drv);

static DEV_INIT(soft_md5_init)
{
  struct soft_md5_private_s *pv;

  dev->status = DEVICE_DRIVER_INIT_FAILED;

  pv = mem_alloc(sizeof (*pv), (mem_scope_sys));

  if (!pv)
    return -ENOMEM;
  memset(pv, 0, sizeof(*pv));

  dev->drv_pv = pv;

  dev_request_delayed_init(&pv->queue, &soft_md5_process);

  dev->drv = &soft_md5_drv;
  dev->status = DEVICE_DRIVER_INIT_DONE;
  return 0;

err_mem:
  mem_free(pv);
  return -1;
}

static DEV_CLEANUP(soft_md5_cleanup)
{
  struct soft_md5_private_s  *pv = dev->drv_pv;

  mem_free(pv);
}
