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

void efm32_aes_ocb_key(struct efm32_aes_context_s * __restrict__ actx,
                       const struct dev_crypto_context_s * __restrict__ ctx)
{
  uint32_t ctrl = EFM32_AES_CTRL_DATASTART | EFM32_AES_CTRL_DECRYPT(ENCRYPTION);

#ifdef CONFIG_DRIVER_EFM32_AES_KEY256
  if (ctx->key_len == 32)
    {
      ctrl |= EFM32_AES_CTRL_AES256;
      efm32_aes_load_key256(ctx->key_data);
    }
  else
#endif
    efm32_aes_load_key128l(ctx->key_data);
  cpu_mem_write_32(EFM32_AES_ADDR + EFM32_AES_CTRL_ADDR, ctrl);

  cpu_mem_write_32(EFM32_AES_ADDR + EFM32_AES_DATA_ADDR, 0);
  cpu_mem_write_32(EFM32_AES_ADDR + EFM32_AES_DATA_ADDR, 0);
  cpu_mem_write_32(EFM32_AES_ADDR + EFM32_AES_DATA_ADDR, 0);
  cpu_mem_write_32(EFM32_AES_ADDR + EFM32_AES_DATA_ADDR, 0);
  efm32_aes_wait();

  actx->ocb_l32[0][3] = endian_be32(cpu_mem_read_32(EFM32_AES_ADDR + EFM32_AES_DATA_ADDR));
  actx->ocb_l32[0][2] = endian_be32(cpu_mem_read_32(EFM32_AES_ADDR + EFM32_AES_DATA_ADDR));
  actx->ocb_l32[0][1] = endian_be32(cpu_mem_read_32(EFM32_AES_ADDR + EFM32_AES_DATA_ADDR));
  actx->ocb_l32[0][0] = endian_be32(cpu_mem_read_32(EFM32_AES_ADDR + EFM32_AES_DATA_ADDR));

  EFM32_AES_DEBUG("l0: %P\n", actx->ocb_l, 16);

  uint_fast8_t i;
  for (i = 1; i < EFM32_AES_OCB_L_COUNT + 2; i++)
    {
      /* OCB double */
      uint8_t *dst = actx->ocb_l[i];
      uint8_t *src = actx->ocb_l[i - 1];
      uint_fast8_t i;
      for (i = 0; i < 15; i++)
        dst[i] = (src[i] << 1) | (src[i + 1] >> 7);

      int8_t m = src[0] & 0x80;
      dst[i] = (src[i] << 1) ^ ((m >> 7) & 0x87);

      EFM32_AES_DEBUG("l%u: %P\n", i, dst, 16);
    }
}

void efm32_aes_ocb_nonce(struct efm32_aes_context_s * __restrict__ actx,
                         const struct dev_crypto_rq_s * __restrict__ rq)
{
  const struct dev_crypto_context_s *ctx = rq->ctx;

  uint_fast8_t i;
  assert(ctx->iv_len < 16);
  assert(ctx->auth_len <= 16);

  /* update actx->ocb_nonce */
  uint8_t *n = actx->ocb_nonce;

  n[0] = (ctx->auth_len * 8) << 1;
  for (i = 0; i < 16 - ctx->iv_len - 1; i++)
    n[i + 1] = 0;
  n[i] |= 0x01;
  for (i++; i < 16; i++)
    n[i] = rq->iv_ctr[i + ctx->iv_len - 16];

  EFM32_AES_DEBUG("n: %P\n", n, 16);

  /* update actx->ocb_stretch */
  uint32_t ctrl = EFM32_AES_CTRL_DATASTART | EFM32_AES_CTRL_DECRYPT(ENCRYPTION);

#ifdef CONFIG_DRIVER_EFM32_AES_KEY256
  if (ctx->key_len == 32)
    {
      ctrl |= EFM32_AES_CTRL_AES256;
      efm32_aes_load_key256(ctx->key_data);
    }
  else
#endif
    efm32_aes_load_key128l(ctx->key_data);
  cpu_mem_write_32(EFM32_AES_ADDR + EFM32_AES_CTRL_ADDR, ctrl);

  cpu_mem_write_32(EFM32_AES_ADDR + EFM32_AES_DATA_ADDR, endian_be32_na_load(n + 12) & 0xffffffc0);
  cpu_mem_write_32(EFM32_AES_ADDR + EFM32_AES_DATA_ADDR, endian_be32_na_load(n + 8));
  cpu_mem_write_32(EFM32_AES_ADDR + EFM32_AES_DATA_ADDR, endian_be32_na_load(n + 4));
  cpu_mem_write_32(EFM32_AES_ADDR + EFM32_AES_DATA_ADDR, endian_be32_na_load(n + 0));
  efm32_aes_wait();

  n = actx->ocb_stretch;
  endian_be32_na_store(n + 12, cpu_mem_read_32(EFM32_AES_ADDR + EFM32_AES_DATA_ADDR));
  endian_be32_na_store(n + 8,  cpu_mem_read_32(EFM32_AES_ADDR + EFM32_AES_DATA_ADDR));
  endian_be32_na_store(n + 4,  cpu_mem_read_32(EFM32_AES_ADDR + EFM32_AES_DATA_ADDR));
  endian_be32_na_store(n + 0,  cpu_mem_read_32(EFM32_AES_ADDR + EFM32_AES_DATA_ADDR));

  for (i = 0; i < 8; i++)
    n[i + 16] = n[i] ^ n[i + 1];

  EFM32_AES_DEBUG("s: %P\n", n, 24);
}

static inline uint_fast8_t efm32_aes_ocb_ntz(size_t i)
{
#if CONFIG_DRIVER_EFM32_AES_OCB3_MAXBLOCKS < 15
  return (0x0484c484 >> (i << 1)) & 3;
#else
  return bit_ctz(i + 1);
#endif
}

#ifdef CONFIG_DRIVER_EFM32_AES_AD
void efm32_aes_ocb_hash(struct efm32_aes_context_s * __restrict__ actx,
                        struct dev_crypto_rq_s * __restrict__ rq, const uint8_t *key,
                        uint32_t tag[4])
{
  uint32_t k0, k1, k2, k3;
  uint32_t o0, o1, o2, o3;

  const uint8_t *in = rq->ad;
  size_t i, m = rq->ad_len >> 4, r = rq->ad_len & 15;

  o0 = o1 = o2 = o3 = 0;
  k0 = k1 = k2 = k3 = 0;

  union {
    uint8_t  p8[16];
    uint32_t p32[4];
  }          p;

  for (i = 0; i < m; i++)
    {
      uint_fast8_t z = 2 + efm32_aes_ocb_ntz(i);

      o0 ^= endian_be32(actx->ocb_l32[z][3]);
      o1 ^= endian_be32(actx->ocb_l32[z][2]);
      o2 ^= endian_be32(actx->ocb_l32[z][1]);
      o3 ^= endian_be32(actx->ocb_l32[z][0]);

      EFM32_AES_KEY_RELOAD(key);
      cpu_mem_write_32(EFM32_AES_ADDR + EFM32_AES_DATA_ADDR, endian_be32_na_load(in + 12) ^ o0);
      cpu_mem_write_32(EFM32_AES_ADDR + EFM32_AES_DATA_ADDR, endian_be32_na_load(in + 8)  ^ o1);
      cpu_mem_write_32(EFM32_AES_ADDR + EFM32_AES_DATA_ADDR, endian_be32_na_load(in + 4)  ^ o2);
      cpu_mem_write_32(EFM32_AES_ADDR + EFM32_AES_DATA_ADDR, endian_be32_na_load(in + 0)  ^ o3);
      efm32_aes_wait();

      k0 ^= cpu_mem_read_32(EFM32_AES_ADDR + EFM32_AES_DATA_ADDR);
      k1 ^= cpu_mem_read_32(EFM32_AES_ADDR + EFM32_AES_DATA_ADDR);
      k2 ^= cpu_mem_read_32(EFM32_AES_ADDR + EFM32_AES_DATA_ADDR);
      k3 ^= cpu_mem_read_32(EFM32_AES_ADDR + EFM32_AES_DATA_ADDR);

      in += 16;
    }

  EFM32_AES_DEBUG("ak: %08x %08x %08x %08x\n", k3, k2, k1, k0);

  if (r)
    {
      p.p32[0] = p.p32[1] = p.p32[2] = p.p32[3] = 0;
      for (i = 0; i < r; i++)
        p.p8[i] = in[i];
      p.p8[i] = 0x80;

      EFM32_AES_KEY_RELOAD(key);
      cpu_mem_write_32(EFM32_AES_ADDR + EFM32_AES_DATA_ADDR, o0 ^ endian_be32(actx->ocb_l32[0][3] ^ p.p32[3]));
      cpu_mem_write_32(EFM32_AES_ADDR + EFM32_AES_DATA_ADDR, o1 ^ endian_be32(actx->ocb_l32[0][2] ^ p.p32[2]));
      cpu_mem_write_32(EFM32_AES_ADDR + EFM32_AES_DATA_ADDR, o2 ^ endian_be32(actx->ocb_l32[0][1] ^ p.p32[1]));
      cpu_mem_write_32(EFM32_AES_ADDR + EFM32_AES_DATA_ADDR, o3 ^ endian_be32(actx->ocb_l32[0][0] ^ p.p32[0]));
      efm32_aes_wait();

      k0 ^= cpu_mem_read_32(EFM32_AES_ADDR + EFM32_AES_DATA_ADDR);
      k1 ^= cpu_mem_read_32(EFM32_AES_ADDR + EFM32_AES_DATA_ADDR);
      k2 ^= cpu_mem_read_32(EFM32_AES_ADDR + EFM32_AES_DATA_ADDR);
      k3 ^= cpu_mem_read_32(EFM32_AES_ADDR + EFM32_AES_DATA_ADDR);
    }

  EFM32_AES_DEBUG("rak: %08x %08x %08x %08x\n", k3, k2, k1, k0);

  tag[3] ^= endian_be32(k0);
  tag[2] ^= endian_be32(k1);
  tag[1] ^= endian_be32(k2);
  tag[0] ^= endian_be32(k3);
}
#endif

void efm32_aes_ocb_encrypt(struct efm32_aes_context_s * __restrict__ actx,
                           struct dev_crypto_rq_s * __restrict__ rq,
                           const uint8_t * __restrict__ key)
{
  const struct dev_crypto_context_s *ctx = rq->ctx;

  uint32_t k0, k1, k2, k3;
  uint32_t o0, o1, o2, o3;
  uint32_t m0, m1, m2, m3;

  union {
    uint8_t  p8[16];
    uint32_t p32[4];
  }          p;

  const uint8_t *in = rq->in;
  uint8_t *out = rq->out;
  size_t i, m = rq->len >> 4, r = rq->len & 15;

  if (m > CONFIG_DRIVER_EFM32_AES_OCB3_MAXBLOCKS)
    return;

  const uint32_t *s = actx->ocb_s32;
  uint_fast8_t b = actx->ocb_nonce[15] & 0x3f;
  uint_fast8_t b8 = b >> 5;
  uint_fast8_t b7 = 32 - (b & 31);
  uint_fast8_t b4 = (b + 31) >> 5;
  uint_fast8_t b5 = (32 - b) & 31;
  o0 = ((uint64_t)endian_be32(s[2 + b4]) << 32 | endian_be32(s[3 + b4])) >> b5;
  o1 = ((uint64_t)endian_be32(s[1 + b4]) << 32 | endian_be32(s[2 + b4])) >> b5;
  o2 = ((uint64_t)endian_be32(s[1 + b8]) << 32 | endian_be32(s[2 + b8])) >> b7;
  o3 = ((uint64_t)endian_be32(s[0 + b8]) << 32 | endian_be32(s[1 + b8])) >> b7;

  EFM32_AES_DEBUG("o: %08x %08x %08x %08x\n", o3, o2, o1, o0);

  k0 = k1 = k2 = k3 = 0;

  uint32_t ctrl = EFM32_AES_CTRL_DATASTART | EFM32_AES_CTRL_DECRYPT(ENCRYPTION);
  EFM32_AES_KEY_CTRL(key, ctrl);
  cpu_mem_write_32(EFM32_AES_ADDR + EFM32_AES_CTRL_ADDR, ctrl);

  for (i = 0; i < m; i++)
    {
      uint_fast8_t z = 2 + efm32_aes_ocb_ntz(i);

      o0 ^= endian_be32(actx->ocb_l32[z][3]);
      o1 ^= endian_be32(actx->ocb_l32[z][2]);
      o2 ^= endian_be32(actx->ocb_l32[z][1]);
      o3 ^= endian_be32(actx->ocb_l32[z][0]);

      EFM32_AES_DEBUG("o2: %08x %08x %08x %08x\n", o3, o2, o1, o0);

      m0 = endian_be32_na_load(in + 12);
      m1 = endian_be32_na_load(in + 8);
      m2 = endian_be32_na_load(in + 4);
      m3 = endian_be32_na_load(in + 0);

      EFM32_AES_DEBUG("m: %08x %08x %08x %08x\n", m3, m2, m1, m0);

      EFM32_AES_KEY_RELOAD(key);
      cpu_mem_write_32(EFM32_AES_ADDR + EFM32_AES_DATA_ADDR, m0 ^ o0);
      cpu_mem_write_32(EFM32_AES_ADDR + EFM32_AES_DATA_ADDR, m1 ^ o1);
      cpu_mem_write_32(EFM32_AES_ADDR + EFM32_AES_DATA_ADDR, m2 ^ o2);
      cpu_mem_write_32(EFM32_AES_ADDR + EFM32_AES_DATA_ADDR, m3 ^ o3);
      efm32_aes_wait();

      endian_be32_na_store(out + 12, o0 ^ cpu_mem_read_32(EFM32_AES_ADDR + EFM32_AES_DATA_ADDR));
      endian_be32_na_store(out + 8,  o1 ^ cpu_mem_read_32(EFM32_AES_ADDR + EFM32_AES_DATA_ADDR));
      endian_be32_na_store(out + 4,  o2 ^ cpu_mem_read_32(EFM32_AES_ADDR + EFM32_AES_DATA_ADDR));
      endian_be32_na_store(out + 0,  o3 ^ cpu_mem_read_32(EFM32_AES_ADDR + EFM32_AES_DATA_ADDR));

      EFM32_AES_DEBUG("out: %P\n", out, 16);

      k0 ^= m0;
      k1 ^= m1;
      k2 ^= m2;
      k3 ^= m3;

      EFM32_AES_DEBUG("k: %08x %08x %08x %08x\n", k3, k2, k1, k0);

      out += 16;
      in += 16;
    }

  EFM32_AES_DEBUG("om: %08x %08x %08x %08x\n", o3, o2, o1, o0);

  if (r)
    {
      p.p32[0] = p.p32[1] = p.p32[2] = p.p32[3] = 0;
      for (i = 0; i < r; i++)
        p.p8[i] = in[i];
      p.p8[i] = 0x80;

      EFM32_AES_DEBUG("rp: %P\n", p.p8, 16);

      k0 ^= endian_be32(p.p32[3]);
      k1 ^= endian_be32(p.p32[2]);
      k2 ^= endian_be32(p.p32[1]);
      k3 ^= endian_be32(p.p32[0]);

      EFM32_AES_DEBUG("rk: %08x %08x %08x %08x\n", k3, k2, k1, k0);

      o0 ^= endian_be32(actx->ocb_l32[0][3]);
      o1 ^= endian_be32(actx->ocb_l32[0][2]);
      o2 ^= endian_be32(actx->ocb_l32[0][1]);
      o3 ^= endian_be32(actx->ocb_l32[0][0]);

      EFM32_AES_DEBUG("ro: %08x %08x %08x %08x\n", o3, o2, o1, o0);

      EFM32_AES_KEY_RELOAD(key);
      cpu_mem_write_32(EFM32_AES_ADDR + EFM32_AES_DATA_ADDR, o0);
      cpu_mem_write_32(EFM32_AES_ADDR + EFM32_AES_DATA_ADDR, o1);
      cpu_mem_write_32(EFM32_AES_ADDR + EFM32_AES_DATA_ADDR, o2);
      cpu_mem_write_32(EFM32_AES_ADDR + EFM32_AES_DATA_ADDR, o3);
      efm32_aes_wait();

      p.p8[r] ^= 0x80;
      p.p32[3] ^= endian_be32(cpu_mem_read_32(EFM32_AES_ADDR + EFM32_AES_DATA_ADDR));
      p.p32[2] ^= endian_be32(cpu_mem_read_32(EFM32_AES_ADDR + EFM32_AES_DATA_ADDR));
      p.p32[1] ^= endian_be32(cpu_mem_read_32(EFM32_AES_ADDR + EFM32_AES_DATA_ADDR));
      p.p32[0] ^= endian_be32(cpu_mem_read_32(EFM32_AES_ADDR + EFM32_AES_DATA_ADDR));

      EFM32_AES_DEBUG("rp2: %P\n", p.p8, 16);

      for (i = 0; i < r; i++)
        out[i] = p.p8[i];
    }

  EFM32_AES_KEY_RELOAD(key);
  cpu_mem_write_32(EFM32_AES_ADDR + EFM32_AES_DATA_ADDR, endian_be32(actx->ocb_l32[1][3]) ^ o0 ^ k0);
  cpu_mem_write_32(EFM32_AES_ADDR + EFM32_AES_DATA_ADDR, endian_be32(actx->ocb_l32[1][2]) ^ o1 ^ k1);
  cpu_mem_write_32(EFM32_AES_ADDR + EFM32_AES_DATA_ADDR, endian_be32(actx->ocb_l32[1][1]) ^ o2 ^ k2);
  cpu_mem_write_32(EFM32_AES_ADDR + EFM32_AES_DATA_ADDR, endian_be32(actx->ocb_l32[1][0]) ^ o3 ^ k3);
  efm32_aes_wait();

  p.p32[3] = endian_be32(cpu_mem_read_32(EFM32_AES_ADDR + EFM32_AES_DATA_ADDR));
  p.p32[2] = endian_be32(cpu_mem_read_32(EFM32_AES_ADDR + EFM32_AES_DATA_ADDR));
  p.p32[1] = endian_be32(cpu_mem_read_32(EFM32_AES_ADDR + EFM32_AES_DATA_ADDR));
  p.p32[0] = endian_be32(cpu_mem_read_32(EFM32_AES_ADDR + EFM32_AES_DATA_ADDR));

  EFM32_AES_DEBUG("auth: %P\n", p.p8, 16);

#ifdef CONFIG_DRIVER_EFM32_AES_AD
  if (rq->ad_len)
    {
      efm32_aes_ocb_hash(actx, rq, key, p.p32);
      EFM32_AES_DEBUG("adauth: %P\n", p.p8, 16);
    }
#endif

  memcpy(rq->auth, p.p8, ctx->auth_len);

  rq->error = 0;
}

void efm32_aes_ocb_decrypt(struct efm32_aes_context_s * __restrict__ actx,
                           struct dev_crypto_rq_s * __restrict__ rq,
                           const uint8_t * __restrict__ key)
{
  const struct dev_crypto_context_s *ctx = rq->ctx;

  uint32_t k0, k1, k2, k3;
  uint32_t o0, o1, o2, o3;
  uint32_t m0, m1, m2, m3;

  union {
    uint8_t  p8[16];
    uint32_t p32[4];
  }          p;

  const uint8_t *in = rq->in;
  uint8_t *out = rq->out;
  size_t i, m = rq->len >> 4, r = rq->len & 15;

  if (m > CONFIG_DRIVER_EFM32_AES_OCB3_MAXBLOCKS)
    return;

  const uint32_t *s = actx->ocb_s32;
  uint_fast8_t b = actx->ocb_nonce[15] & 0x3f;
  uint_fast8_t b8 = b >> 5;
  uint_fast8_t b7 = 32 - (b & 31);
  uint_fast8_t b4 = (b + 31) >> 5;
  uint_fast8_t b5 = (32 - b) & 31;
  o0 = ((uint64_t)endian_be32(s[2 + b4]) << 32 | endian_be32(s[3 + b4])) >> b5;
  o1 = ((uint64_t)endian_be32(s[1 + b4]) << 32 | endian_be32(s[2 + b4])) >> b5;
  o2 = ((uint64_t)endian_be32(s[1 + b8]) << 32 | endian_be32(s[2 + b8])) >> b7;
  o3 = ((uint64_t)endian_be32(s[0 + b8]) << 32 | endian_be32(s[1 + b8])) >> b7;

  EFM32_AES_DEBUG("o: %08x %08x %08x %08x\n", o3, o2, o1, o0);

  k0 = k1 = k2 = k3 = 0;

  uint32_t ctrl = EFM32_AES_CTRL_DATASTART | EFM32_AES_CTRL_DECRYPT(DECRYPTION);
  EFM32_AES_KEY_CTRL(key, ctrl);
  cpu_mem_write_32(EFM32_AES_ADDR + EFM32_AES_CTRL_ADDR, ctrl);

  for (i = 0; i < m; i++)
    {
      uint_fast8_t z = 2 + efm32_aes_ocb_ntz(i);

      EFM32_AES_KEY_RELOAD(key);

      o0 ^= endian_be32(actx->ocb_l32[z][3]);
      o1 ^= endian_be32(actx->ocb_l32[z][2]);
      o2 ^= endian_be32(actx->ocb_l32[z][1]);
      o3 ^= endian_be32(actx->ocb_l32[z][0]);

      EFM32_AES_DEBUG("o2: %08x %08x %08x %08x\n", o3, o2, o1, o0);

      cpu_mem_write_32(EFM32_AES_ADDR + EFM32_AES_DATA_ADDR, endian_be32_na_load(in + 12) ^ o0);
      cpu_mem_write_32(EFM32_AES_ADDR + EFM32_AES_DATA_ADDR, endian_be32_na_load(in + 8)  ^ o1);
      cpu_mem_write_32(EFM32_AES_ADDR + EFM32_AES_DATA_ADDR, endian_be32_na_load(in + 4)  ^ o2);
      cpu_mem_write_32(EFM32_AES_ADDR + EFM32_AES_DATA_ADDR, endian_be32_na_load(in + 0)  ^ o3);
      efm32_aes_wait();

      m0 = o0 ^ cpu_mem_read_32(EFM32_AES_ADDR + EFM32_AES_DATA_ADDR);
      m1 = o1 ^ cpu_mem_read_32(EFM32_AES_ADDR + EFM32_AES_DATA_ADDR);
      m2 = o2 ^ cpu_mem_read_32(EFM32_AES_ADDR + EFM32_AES_DATA_ADDR);
      m3 = o3 ^ cpu_mem_read_32(EFM32_AES_ADDR + EFM32_AES_DATA_ADDR);

      EFM32_AES_DEBUG("m: %08x %08x %08x %08x\n", m3, m2, m1, m0);

      endian_be32_na_store(out + 12, m0);
      endian_be32_na_store(out + 8,  m1);
      endian_be32_na_store(out + 4,  m2);
      endian_be32_na_store(out + 0,  m3);

      EFM32_AES_DEBUG("out: %P\n", out, 16);

      k0 ^= m0;
      k1 ^= m1;
      k2 ^= m2;
      k3 ^= m3;

      EFM32_AES_DEBUG("k: %08x %08x %08x %08x\n", k3, k2, k1, k0);

      out += 16;
      in += 16;
    }

  ctrl = EFM32_AES_CTRL_DATASTART | EFM32_AES_CTRL_DECRYPT(ENCRYPTION);
  key = ctx->key_data;
#ifdef EFM32_AES_CTRL_KEYBUFEN
  if (ctx->key_len == 16)
    {
      ctrl |= EFM32_AES_CTRL_KEYBUFEN;
      efm32_aes_load_key128h(key);
      key = NULL;
    }
# ifdef CONFIG_DRIVER_EFM32_AES_KEY256
  else
    ctrl |= EFM32_AES_CTRL_AES256;
# endif
#endif
  cpu_mem_write_32(EFM32_AES_ADDR + EFM32_AES_CTRL_ADDR, ctrl);

  EFM32_AES_DEBUG("om: %08x %08x %08x %08x\n", o3, o2, o1, o0);

  if (r)
    {
      EFM32_AES_KEY_RELOAD(key);

      EFM32_AES_DEBUG("rp: %P\n", p.p8, 16);

      o0 ^= endian_be32(actx->ocb_l32[0][3]);
      o1 ^= endian_be32(actx->ocb_l32[0][2]);
      o2 ^= endian_be32(actx->ocb_l32[0][1]);
      o3 ^= endian_be32(actx->ocb_l32[0][0]);

      EFM32_AES_DEBUG("ro: %08x %08x %08x %08x\n", o3, o2, o1, o0);

      cpu_mem_write_32(EFM32_AES_ADDR + EFM32_AES_DATA_ADDR, o0);
      cpu_mem_write_32(EFM32_AES_ADDR + EFM32_AES_DATA_ADDR, o1);
      cpu_mem_write_32(EFM32_AES_ADDR + EFM32_AES_DATA_ADDR, o2);
      cpu_mem_write_32(EFM32_AES_ADDR + EFM32_AES_DATA_ADDR, o3);
      efm32_aes_wait();

      p.p32[3] = endian_be32(cpu_mem_read_32(EFM32_AES_ADDR + EFM32_AES_DATA_ADDR));
      p.p32[2] = endian_be32(cpu_mem_read_32(EFM32_AES_ADDR + EFM32_AES_DATA_ADDR));
      p.p32[1] = endian_be32(cpu_mem_read_32(EFM32_AES_ADDR + EFM32_AES_DATA_ADDR));
      p.p32[0] = endian_be32(cpu_mem_read_32(EFM32_AES_ADDR + EFM32_AES_DATA_ADDR));

      for (i = 0; i < r; i++)
        out[i] = p.p8[i] ^= in[i];
      p.p8[i++] = 0x80;
      for (; i < 16; i++)
        p.p8[i] = 0;

      EFM32_AES_DEBUG("rp2: %P\n", p.p8, 16);

      k0 ^= endian_be32(p.p32[3]);
      k1 ^= endian_be32(p.p32[2]);
      k2 ^= endian_be32(p.p32[1]);
      k3 ^= endian_be32(p.p32[0]);

      EFM32_AES_DEBUG("rk: %08x %08x %08x %08x\n", k3, k2, k1, k0);
    }

  EFM32_AES_KEY_RELOAD(key);

  cpu_mem_write_32(EFM32_AES_ADDR + EFM32_AES_DATA_ADDR, endian_be32(actx->ocb_l32[1][3]) ^ o0 ^ k0);
  cpu_mem_write_32(EFM32_AES_ADDR + EFM32_AES_DATA_ADDR, endian_be32(actx->ocb_l32[1][2]) ^ o1 ^ k1);
  cpu_mem_write_32(EFM32_AES_ADDR + EFM32_AES_DATA_ADDR, endian_be32(actx->ocb_l32[1][1]) ^ o2 ^ k2);
  cpu_mem_write_32(EFM32_AES_ADDR + EFM32_AES_DATA_ADDR, endian_be32(actx->ocb_l32[1][0]) ^ o3 ^ k3);
  efm32_aes_wait();

  p.p32[3] = endian_be32(cpu_mem_read_32(EFM32_AES_ADDR + EFM32_AES_DATA_ADDR));
  p.p32[2] = endian_be32(cpu_mem_read_32(EFM32_AES_ADDR + EFM32_AES_DATA_ADDR));
  p.p32[1] = endian_be32(cpu_mem_read_32(EFM32_AES_ADDR + EFM32_AES_DATA_ADDR));
  p.p32[0] = endian_be32(cpu_mem_read_32(EFM32_AES_ADDR + EFM32_AES_DATA_ADDR));

  EFM32_AES_DEBUG("auth: %P\n", p.p8, 16);

#ifdef CONFIG_DRIVER_EFM32_AES_AD
  if (rq->ad_len)
    {
      efm32_aes_ocb_hash(actx, rq, key, p.p32);
      EFM32_AES_DEBUG("adauth: %P\n", p.p8, 16);
    }
#endif

  uint8_t c = 0;
  for (i = 0; i < ctx->auth_len; i++)
    c = rq->auth[i] ^ p.p8[i];
  rq->error = c ? -EBADDATA : 0;
}

