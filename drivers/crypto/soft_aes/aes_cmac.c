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
uint32_t __lt32(uint32_t a, uint32_t b)
{
  return (a^((a^b)|((a-b)^b))) >> 31;
}

static
void soft_aes_cmac_shit_left(const uint32_t * in, uint32_t * out, uint32_t * ov)
{
  uint_fast8_t i;

  *ov = in[0] >> 31;
  for (i = 0; i < 3; i++)
    out[i] = (in[i] << 1) | (in[i+1] >> 31);
  out[3] = in[3] << 1;
}

static
void soft_aes_gen_subkey(const uint32_t * data, uint32_t * subkey)
{
  uint32_t ov = 0;

  soft_aes_cmac_shit_left(data, subkey, &ov);
  subkey[3] ^= endian_be32(ov * 0x87);
}

void soft_aes_cmac(struct soft_aes_context_s *actx,
                   struct dev_crypto_rq_s *rq)
{
  const struct dev_crypto_context_s *ctx = rq->ctx;
  uint_fast8_t i;
  size_t len = rq->len;
  const uint8_t *in = rq->in;
  uint8_t *out;

  if (ctx->iv_len != 16)
    return;
  if (ctx->auth_len != 16)
    return;

  uint32_t iv[4];
  for (i = 0; i < 4; i++)
    iv[i] = endian_be32_na_load(rq->iv_ctr + i * 4);

  uint32_t k1[4]     = { 0 }, k2[4];
  uint8_t  blank[17] = { 0 }; /* Need one extra byte for padding overflow. */

  /* Sub key generation */
  soft_aes_encrypt(actx, k1);
  soft_aes_gen_subkey(k1, k1);
  soft_aes_gen_subkey(k1, k2);

  /* N-1th blocks */
  for (; len > 16; len -= 16)
    {
      for (i = 0; i < 4; i++)
        iv[i] ^= endian_be32_na_load(in + i * 4);
      soft_aes_encrypt(actx, iv);
      in += 16;
    }

  /* Select the right subkey. */
  uint32_t const lt16_mask = __lt32(i, 16) * (uint32_t)-1;
  /* If not equal to 16, the block is < 16 bytes */
  uint32_t const eq16_mask = ~lt16_mask;

  for (i = 0; i < 4; i++)
    k1[i] = (k1[i] & eq16_mask) | (k2[i] & lt16_mask);

  /* Padding */
  for (i = 0; len-- > 0; i++)
    blank[i] = in[i];
  blank[i] = 0x80;

  /* Nth block */
  in = blank;
  for (i = 0; i < 4; ++i)
    iv[i] ^= endian_be32_na_load(in + i * 4) ^ endian_be32(k1[i]);
  soft_aes_encrypt(actx, iv);

  rq->err = 0;

  if (rq->op & DEV_CRYPTO_FINALIZE)
    {
      if (rq->op & DEV_CRYPTO_INVERSE)
        {
          if (dev_crypto_memcmp(iv, rq->auth, 16))
            rq->err = -EBADDATA;
          return;
        }
      else
        {
          out = rq->auth;
        }
    }
  else
    {
      out = rq->iv_ctr;
    }

  for (i = 0; i < 4; i++)
    endian_be32_na_store(out + i * 4, iv[i]);
}

