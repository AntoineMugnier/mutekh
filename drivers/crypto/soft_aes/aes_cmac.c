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

/* Follows AES-CMAC defined in RFC4493. */

#include "aes.h"

static inline
void soft_aes_cmac_shift_left(const uint32_t * in, uint32_t * out, uint32_t * ov)
{
  int_fast8_t i;

  uint32_t s = 0;

  for (i = 3; i >= 0; --i)
    {
      uint32_t tmp = in[i];
      out[i] = (tmp << 1) | s;
      s = tmp >> 31;
    }
  *ov = s;
}

static inline
void soft_aes_gen_subkey(const uint32_t * data, uint32_t * subkey)
{
  uint32_t ov = 0;

  soft_aes_cmac_shift_left(data, subkey, &ov);
  subkey[3] ^= (uint32_t)((ov ^ 1) - 1) & 0x87;
}

void soft_aes_cmac(struct soft_aes_context_s *actx,
                   struct dev_crypto_rq_s *rq)
{
  const struct dev_crypto_context_s *ctx = rq->ctx;
  uint_fast8_t i;
  size_t len = rq->len;
  const uint8_t *in = rq->in;

  /* Does not support AES-CMAC block padding if not on finalize operation (i.e.
     at the end of the data buffer). */
  if (!(rq->op & DEV_CRYPTO_FINALIZE) && (rq->len & 0xf))
    return;
  if (ctx->iv_len != 16)
    return;
  if (ctx->auth_len > 16 || ctx->auth_len & 0x3)
    return;

  uint32_t iv[4];
  for (i = 0; i < 4; i++)
    iv[i] = endian_be32_na_load(rq->iv_ctr + i * 4);

  /* N-1th blocks */
  for (; len > 16; len -= 16)
    {
      for (i = 0; i < 4; i++)
        iv[i] ^= endian_be32_na_load(in + i * 4);
      soft_aes_encrypt(actx, iv);
      in += 16;
    }

  rq->error = 0;

  if (rq->op & DEV_CRYPTO_FINALIZE)
    {
      uint32_t subkey[4] = { 0 };
      uint8_t  blank[16] = { 0 };

      /* Generate K1 subkey */
      soft_aes_encrypt(actx, subkey);
      soft_aes_gen_subkey(subkey, subkey);

      /* If last block is not 16-byte long, add padding */
      if (len < 16)
        {
          // Generate K2 subkey for padded data
          soft_aes_gen_subkey(subkey, subkey);

          // Pad remaing bytes, blank trailing bytes are already 0
          for (i = 0; len-- > 0; i++)
            blank[i] = in[i];
          blank[i] = 0x80;

          // Use padded block
          in = blank;
        }

      /* Nth block */
      for (i = 0; i < 4; ++i)
        iv[i] ^= endian_be32_na_load(in + i * 4) ^ subkey[i];
      soft_aes_encrypt(actx, iv);

      if (rq->op & DEV_CRYPTO_INVERSE)
        {
          uint32_t c = 0;

          for (i = 0; i < ctx->auth_len/4; ++i)
            c |= endian_be32_na_load(rq->auth + i * 4) ^ iv[i];

          if (c)
            rq->error = -EBADDATA;
        }
      else
        {
          for (i = 0; i < ctx->auth_len/4; ++i)
            endian_be32_na_store(rq->auth + i * 4, iv[i]);
        }
    }
  else
    {
      /* Nth block */
      for (i = 0; i < 4; ++i)
        iv[i] ^= endian_be32_na_load(in + i * 4);
      soft_aes_encrypt(actx, iv);

      for (i = 0; i < 4; i++)
        endian_be32_na_store(rq->iv_ctr + i * 4, iv[i]);
    }
}

