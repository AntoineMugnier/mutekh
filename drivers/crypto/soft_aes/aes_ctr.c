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

void soft_aes_ctr(struct soft_aes_context_s *actx,
                  struct dev_crypto_rq_s *rq)
{
  const struct dev_crypto_context_s *ctx = rq->ctx;
  uint_fast8_t i;
  size_t l = rq->len;
  const uint8_t *in = rq->in;
  uint8_t *out = rq->out;

  uint32_t iv[4];

  union {
    uint8_t  p8[16];
    uint32_t p32[4];
  }          p;

  memcpy(p.p8 + 16 - ctx->iv_len, rq->iv_ctr, ctx->iv_len);
  memset(p.p8, 0, 16 - ctx->iv_len);
  for (i = 0; i < 4; i++)
    iv[i] = endian_be32(p.p32[i]);

  while (1)
    {
      for (i = 0; i < 4; i++)
        p.p32[i] = iv[i];
      soft_aes_encrypt(actx, p.p32);
      for (i = 0; i < 4; i++)
        p.p32[i] = endian_be32(p.p32[i]);

      if (l < 16)
        {
          for (i = 0; i < l; i++)
            out[i] = in[i] ^ p.p8[i];
          break;
        }

      for (i = 0; i < 16; i++)
        out[i] = in[i] ^ p.p8[i];

      uint64_t t = 0x100000000ULL;
      for (i = 0; i < 4; i++)
        iv[3-i] = t = (t >> 32) + iv[3-i];

      out += 16;
      in += 16;
      l -= 16;
    }

  if (rq->op & DEV_CRYPTO_FINALIZE)
    {
      for (i = 0; i < 4; i++)
        p.p32[i] = endian_be32(iv[i]);
      memcpy(rq->iv_ctr, p.p8 + 16 - ctx->iv_len, ctx->iv_len);
    }

  rq->err = 0;
}

