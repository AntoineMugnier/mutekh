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

static void soft_aes_ocb_key(struct soft_aes_context_s * __restrict__ actx,
                             const struct dev_crypto_context_s * __restrict__ ctx)
{
  uint_fast8_t i, j;

  for (j = 0; j < 4; j++)
    actx->ocb.l32[0][j] = 0;

  soft_aes_encrypt(actx, actx->ocb.l32[0]);

  for (i = 1; i < AES_OCB_L_COUNT + 2; i++)
    {
      /* OCB double */
      uint32_t *dst = actx->ocb.l32[i];
      uint32_t *src = actx->ocb.l32[i - 1];
      for (j = 0; j < 3; j++)
        dst[j] = (src[j] << 1) | (src[j + 1] >> 31);

      uint32_t m = (src[0] & 0x80000000) >> 24;
      m |= (m >> 5) | (m >> 6) | (m >> 7);
      dst[3] = (src[3] << 1) ^ m;
    }
}

static void soft_aes_ocb_nonce(struct soft_aes_context_s * __restrict__ actx,
                               const struct dev_crypto_rq_s * __restrict__ rq)
{
  const struct dev_crypto_context_s * __restrict__ ctx = rq->ctx;

  uint_fast8_t i;

  /* update actx->ocb_nonce */
  uint8_t *n = actx->ocb.nonce;

  n[0] = (ctx->auth_len * 8) << 1;
  for (i = 0; i < 16 - ctx->iv_len - 1; i++)
    n[i + 1] = 0;
  n[i] |= 0x01;
  for (i++; i < 16; i++)
    n[i] = rq->iv_ctr[i + ctx->iv_len - 16];

  /* update actx->ocb_stretch */
  uint32_t *s = actx->ocb.s32;

  s[0] = endian_be32_na_load(n + 0);
  s[1] = endian_be32_na_load(n + 4);
  s[2] = endian_be32_na_load(n + 8);
  s[3] = endian_be32_na_load(n + 12) & 0xffffffc0;

  soft_aes_encrypt(actx, s);

  s[4] = s[0] ^ (s[0] << 8) ^ (s[1] >> 24);
  s[5] = s[1] ^ (s[1] << 8) ^ (s[2] >> 24);
}

static inline uint_fast8_t soft_aes_ocb_ntz(size_t i)
{
#if CONFIG_DRIVER_SOFT_AES_OCB3_MAXBLOCKS < 15
  return (0x0484c484 >> (i << 1)) & 3;
#else
  return bit_ctz(i + 1);
#endif
}

static void soft_aes_ocb_offset(uint32_t o[4], const uint32_t s[6], uint8_t b)
{
  uint_fast8_t b8 = b >> 5;
  uint_fast8_t b7 = 32 - (b & 31);
  uint_fast8_t b4 = (b + 31) >> 5;
  uint_fast8_t b5 = (32 - b) & 31;
  o[0] = (((uint64_t)s[0 + b8] << 32) | s[1 + b8]) >> b7;
  o[1] = (((uint64_t)s[1 + b8] << 32) | s[2 + b8]) >> b7;
  o[2] = (((uint64_t)s[1 + b4] << 32) | s[2 + b4]) >> b5;
  o[3] = (((uint64_t)s[2 + b4] << 32) | s[3 + b4]) >> b5;
}

static inline void soft_aes_ocb_cipher(struct soft_aes_context_s * __restrict__ actx,
                                       struct soft_aes_state_s * __restrict__ st,
                                       struct dev_crypto_rq_s * __restrict__ rq,
                                       uint32_t k[4])
{
  uint32_t m[4], o[4];
  size_t i = 0, l = rq->len;
  const uint8_t *in = rq->in;
  uint8_t *out = rq->out;
  uint_fast8_t j;

  if (rq->op & DEV_CRYPTO_INIT)
    {
      /* init OCB mode variables */
      soft_aes_ocb_offset(o, actx->ocb.s32, actx->ocb.nonce[15] & 0x3f);
      k[0] = k[1] = k[2] = k[3] = 0;
    }
  else
    {
      /* restore OCB mode variables from state */
      for (j = 0; j < 4; j++)
        {
          o[j] = st->ocb.offset[j];
          k[j] = st->ocb.sum[j];
        }
      i = st->ocb.i;
    }

  if (rq->op & DEV_CRYPTO_INVERSE)
    {
      /* decrypt blocks */
      for (; l >= 16; l -= 16)
        {
          uint_fast8_t z = 2 + soft_aes_ocb_ntz(i++);

          for (j = 0; j < 4; j++)
            {
              o[j] ^= actx->ocb.l32[z][j];
              m[j] = o[j] ^ endian_be32_na_load(in + j * 4);
            }
          soft_aes_decrypt(actx, m);
          for (j = 0; j < 4; j++)
            {
              k[j] ^= m[j] ^= o[j];
              endian_be32_na_store(out + j * 4, m[j]);
            }

          out += 16;
          in += 16;
        }
    }
  else
    {
      /* encrypt blocks */
      for (; l >= 16; l -= 16)
        {
          uint_fast8_t z = 2 + soft_aes_ocb_ntz(i++);

          for (j = 0; j < 4; j++)
            {
              o[j] ^= actx->ocb.l32[z][j];
              k[j] ^= m[j] = endian_be32_na_load(in + j * 4);
              m[j] ^= o[j];
            }
          soft_aes_encrypt(actx, m);
          for (j = 0; j < 4; j++)
            endian_be32_na_store(out + j * 4, m[j] ^ o[j]);

          out += 16;
          in += 16;
        }
    }

  if (rq->op & DEV_CRYPTO_FINALIZE)
    {
      if (l > 0)
        {
          union {
            uint8_t  p8[16];
            uint32_t p32[4];
          }          p;

          memcpy(p.p8, in, l);
          memset(p.p8 + l, 0, sizeof(p.p32) - l);

          if (rq->op & DEV_CRYPTO_INVERSE)
            {
              /* decrypt residue */
              for (j = 0; j < 4; j++)
                m[j] = o[j] ^= actx->ocb.l32[0][j];
              soft_aes_encrypt(actx, m);
              for (j = 0; j < 4; j++)
                p.p32[j] ^= endian_be32(m[j]);
              memcpy(out, p.p8, l);
              p.p8[l] = 0x80;
              memset(p.p8 + l + 1, 0, 15 - l);
              for (j = 0; j < 4; j++)
                k[j] ^= endian_be32(p.p32[j]);
            }
          else
            {
              /* encrypt residue */
              p.p8[l] = 0x80;
              memset(p.p8 + l + 1, 0, 15 - l);
              for (j = 0; j < 4; j++)
                {
                  k[j] ^= endian_be32(p.p32[j]);
                  m[j] = o[j] ^= actx->ocb.l32[0][j];
                }
              soft_aes_encrypt(actx, m);
              for (j = 0; j < 4; j++)
                p.p32[j] ^= endian_be32(m[j]);
              p.p8[l] ^= 0x80;
              memcpy(out, p.p8, l);
            }
        }

      /* compute auth */
      for (j = 0; j < 4; j++)
        k[j] ^= o[j] ^ actx->ocb.l32[1][j];
      soft_aes_encrypt(actx, k);
    }
  else
    {
      /* save variables and residue to state */
      for (j = 0; j < 4; j++)
        {
          st->ocb.offset[j] = o[j];
          st->ocb.sum[j] = k[j];
        }
    }
}

#ifdef CONFIG_DRIVER_CRYPTO_SOFT_AES_AD
static inline void soft_aes_ocb_ad(struct soft_aes_context_s * __restrict__ actx,
                                   struct soft_aes_state_s * __restrict__ st,
                                   struct dev_crypto_rq_s * __restrict__ rq,
                                   uint32_t sum[4])
{
  uint32_t m[4], k[4], o[4];
  size_t i = 0, l = rq->ad_len;
  const uint8_t *in = rq->ad;
  uint_fast8_t j;

  if (rq->op & DEV_CRYPTO_INIT)
    {
      k[0] = k[1] = k[2] = k[3] = 0;
      o[0] = o[1] = o[2] = o[3] = 0;
    }
  else
    {
      /* restore OCB mode ad variables from state */
      for (j = 0; j < 4; j++)
        {
          o[j] = st->ocb.offset_a[j];
          k[j] = st->ocb.sum_a[j];
        }
      i = st->ocb.i_a;
    }

  /* process blocks */
  for (; l >= 16; l -= 16)
    {
      uint_fast8_t z = 2 + soft_aes_ocb_ntz(i++);

      for (j = 0; j < 4; j++)
        {
          o[j] ^= actx->ocb.l32[z][j];
          m[j] = o[j] ^ endian_be32_na_load(in + j * 4);
        }
      soft_aes_encrypt(actx, m);
      for (j = 0; j < 4; j++)
        k[j] ^= m[j];

      in += 16;
    }

  if (rq->op & DEV_CRYPTO_FINALIZE)
    {
      if (l > 0)
        {
          union {
            uint8_t  p8[16];
            uint32_t p32[4];
          }          p;

          /* process residue */
          memcpy(p.p8, in, l);
          p.p8[l] = 0x80;
          memset(p.p8 + l + 1, 0, 15 - l);

          for (j = 0; j < 4; j++)
            m[j] = o[j] ^ actx->ocb.l32[0][j] ^ endian_be32(p.p32[j]);
          soft_aes_encrypt(actx, m);
          for (j = 0; j < 4; j++)
            k[j] ^= m[j];
        }

      /* merge auth */
      for (j = 0; j < 4; j++)
        sum[j] ^= k[j];
    }
  else
    {
      /* save residue and ad variables to state */
      for (j = 0; j < 4; j++)
        {
          st->ocb.offset_a[j] = o[j];
          st->ocb.sum_a[j] = k[j];
        }
    }
}
#endif

void soft_aes_ocb(struct soft_aes_context_s * __restrict__ actx,
                  struct dev_crypto_rq_s * __restrict__ rq,
                  bool_t ctx_ok)
{
  const struct dev_crypto_context_s * __restrict__ ctx = rq->ctx;
  uint_fast8_t i, j;
  uint8_t c = 0;

  if (!ctx_ok)
    {
      if (ctx->iv_len >= 16 || ctx->auth_len > 16)
        return;
      soft_aes_ocb_key(actx, ctx);
      c = 1;
    }
  else
    {
      /* check nonce change */
      for ((j = 16 - ctx->iv_len), (i = 0); j < 15; i++, j++)
        c |= rq->iv_ctr[i] ^ actx->ocb.nonce[j];
      c |= (rq->iv_ctr[i] ^ actx->ocb.nonce[j]) & 0xc0;
    }

  if (c)
    soft_aes_ocb_nonce(actx, rq);

  uint32_t k[4];

#ifndef CONFIG_DRIVER_CRYPTO_SOFT_AES_AD
  if (rq->ad_len)
    return;
#endif

  soft_aes_ocb_cipher(actx, ctx->state_data, rq, k);
#ifdef CONFIG_DRIVER_CRYPTO_SOFT_AES_AD
  soft_aes_ocb_ad(actx, ctx->state_data, rq, k);
#endif

  if (rq->op & DEV_CRYPTO_FINALIZE)
    {
      union {
        uint8_t  p8[16];
        uint32_t p32[4];
      }          p;

      for (j = 0; j < 4; j++)
        p.p32[j] = endian_be32(k[j]);

      if (rq->op & DEV_CRYPTO_INVERSE)
        {
          rq->error = dev_crypto_memcmp(rq->auth, p.p8, ctx->auth_len) ? -EBADDATA : 0;
        }
      else
        {
          memcpy(rq->auth, p.p8, ctx->auth_len);
          rq->error = 0;
        }
    }
  else
    {
      rq->error = (rq->len | rq->ad_len) & 15 ? -EINVAL : 0;
    }
}

